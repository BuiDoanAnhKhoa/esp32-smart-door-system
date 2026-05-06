#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

// ======================================================
// SYSTEM CONFIG
// ======================================================
#define SERIAL_BAUD 115200
#define WIFI_CHANNEL 1

// Broadcast by default.
// After stable testing, replace this with ESP1 MAC for unicast.
uint8_t PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define UID_TEXT_LEN 32
#define MAX_USERS 10
#define PIN_MAX_LEN 10
#define PIN_MIN_LEN 4

// ======================================================
// ESP-NOW PROTOCOL
// Must match ESP1 exactly
// ======================================================
enum MsgType : uint8_t {
  MSG_NONE = 0,

  // ESP1 -> ESP2
  MSG_CARD_SCANNED = 1,
  MSG_DOOR_OPENED,
  MSG_DOOR_CLOSED,
  MSG_TEMP_REPORT,
  MSG_FIRE_ALARM,

  // ESP2 -> ESP1
  MSG_CMD_OPEN_DOOR = 100,
  MSG_CMD_CLOSE_DOOR,
  MSG_CMD_DENY
};

typedef struct __attribute__((packed)) {
  MsgType type;
  char uid[UID_TEXT_LEN];
  float temperatureC;
  float humidity;
  bool doorOpen;
  bool fireAlarm;
  uint32_t counter;
} EspNowPacket;

// ======================================================
// LCD CONFIG
// ======================================================
#define I2C_SDA 19
#define I2C_SCL 18

#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// ======================================================
// KEYPAD CONFIG
// ======================================================
const byte ROWS = 4;
const byte COLS = 4;

char keyMap[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// User requested keypad GPIO0 -> GPIO7
byte rowPins[ROWS] = {0, 1, 2, 3};
byte colPins[COLS] = {4, 5, 6, 7};

Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);

// ======================================================
// RTOS OBJECTS
// ======================================================
QueueHandle_t qUiEvent;
QueueHandle_t qLcd;
QueueHandle_t qEspNowTx;

SemaphoreHandle_t lcdMutex;

TimerHandle_t pinInputTimeoutTimer;
TimerHandle_t adminTimeoutTimer;
TimerHandle_t lockoutTimer;
TimerHandle_t uiReturnTimer;

// ======================================================
// RAM-ONLY DATABASE
// Reset/EN or power loss clears everything. No Preferences/NVS.
// ======================================================
struct UserRecord {
  char uid[UID_TEXT_LEN];
  char pin[PIN_MAX_LEN + 1];
  bool active;
};

struct RuntimeDatabase {
  bool setupDone;
  char adminPin[PIN_MAX_LEN + 1];
  char adminUid[UID_TEXT_LEN];
  bool adminCanOpenDoor;
  UserRecord users[MAX_USERS];
};

RuntimeDatabase db;

// ======================================================
// UI EVENTS
// ======================================================
enum UiEventType : uint8_t {
  EV_NONE = 0,
  EV_KEY,
  EV_PACKET,
  EV_PIN_TIMEOUT,
  EV_ADMIN_TIMEOUT,
  EV_LOCKOUT_END,
  EV_UI_RETURN
};

typedef struct {
  UiEventType type;
  char key;
  EspNowPacket packet;
} UiEvent;

typedef struct {
  char line1[17];
  char line2[17];
} LcdMessage;

// ======================================================
// UI STATE MACHINE
// ======================================================
enum UiState : uint8_t {
  STATE_FIRST_SET_ADMIN_PIN,
  STATE_FIRST_SCAN_ADMIN_CARD,

  STATE_IDLE,
  STATE_ENTER_USER_PIN,

  STATE_ADMIN_LOGIN,
  STATE_ADMIN_MENU,
  STATE_ADD_USER_WAIT_CARD,
  STATE_ADD_USER_WAIT_PIN,
  STATE_DELETE_USER_WAIT_CARD,

  STATE_ACCESS_GRANTED,
  STATE_ACCESS_DENIED,
  STATE_FIRE_ALARM,
  STATE_LOCKED_OUT
};

UiState state = STATE_IDLE;

char inputBuf[PIN_MAX_LEN + 1] = "";
uint8_t inputLen = 0;

char pendingUid[UID_TEXT_LEN] = "";
char pendingAdminPin[PIN_MAX_LEN + 1] = "";

uint8_t failedAttempts = 0;

uint32_t packetCounter = 0;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

// ======================================================
// HELPERS
// ======================================================
uint32_t nextCounter() {
  uint32_t value;
  portENTER_CRITICAL(&counterMux);
  value = ++packetCounter;
  portEXIT_CRITICAL(&counterMux);
  return value;
}

void safeCopy(char *dst, const char *src, size_t dstSize) {
  if (dstSize == 0) return;
  strncpy(dst, src, dstSize - 1);
  dst[dstSize - 1] = '\0';
}

bool keyIsDigit(char k) {
  return k >= '0' && k <= '9';
}

void clearInput() {
  inputBuf[0] = '\0';
  inputLen = 0;
}

bool appendDigit(char k) {
  if (!keyIsDigit(k)) return false;
  if (inputLen >= PIN_MAX_LEN) return false;

  inputBuf[inputLen++] = k;
  inputBuf[inputLen] = '\0';
  return true;
}

void backspaceInput() {
  if (inputLen == 0) return;

  inputLen--;
  inputBuf[inputLen] = '\0';
}

void makePinMask(char *out, size_t outSize) {
  out[0] = '\0';

  for (uint8_t i = 0; i < inputLen && i < outSize - 1; i++) {
    out[i] = '*';
    out[i + 1] = '\0';
  }
}

void postLcd(const char *line1, const char *line2) {
  LcdMessage msg = {};
  safeCopy(msg.line1, line1, sizeof(msg.line1));
  safeCopy(msg.line2, line2, sizeof(msg.line2));
  if (xQueueSend(qLcd, &msg, 0) != pdTRUE) {
    Serial.println("[ESP2] WARN: qLcd full");
  }
}

void postUiEvent(UiEventType type) {
  UiEvent ev = {};
  ev.type = type;
  if (xQueueSend(qUiEvent, &ev, 0) != pdTRUE) {
    Serial.println("[ESP2] WARN: qUiEvent full");
  }
}

void restartPinTimer() {
  xTimerStop(pinInputTimeoutTimer, 0);
  xTimerStart(pinInputTimeoutTimer, 0);
}

void stopPinTimer() {
  xTimerStop(pinInputTimeoutTimer, 0);
}

void restartAdminTimer() {
  xTimerStop(adminTimeoutTimer, 0);
  xTimerStart(adminTimeoutTimer, 0);
}

void stopAdminTimer() {
  xTimerStop(adminTimeoutTimer, 0);
}

void startUiReturnTimer(uint32_t ms = 1600) {
  xTimerStop(uiReturnTimer, 0);
  xTimerChangePeriod(uiReturnTimer, pdMS_TO_TICKS(ms), 0);
  xTimerStart(uiReturnTimer, 0);
}

// ======================================================
// RAM-ONLY DATABASE HELPERS
// ======================================================
void dbResetRuntime() {
  memset(&db, 0, sizeof(db));
  db.setupDone = false;
  db.adminCanOpenDoor = true;
}

bool dbInitialized() {
  return db.setupDone;
}

void dbSetInitialized(bool value) {
  db.setupDone = value;
}

bool dbIsAdminPin(const char *pin) {
  return db.adminPin[0] != '\0' && strcmp(db.adminPin, pin) == 0;
}

bool dbIsAdminUid(const char *uid) {
  return db.adminUid[0] != '\0' && strcmp(db.adminUid, uid) == 0;
}

bool dbUserUidExists(const char *uid) {
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!db.users[i].active) continue;

    if (strcmp(db.users[i].uid, uid) == 0) {
      return true;
    }
  }

  return false;
}

bool dbUserPinExists(const char *pin) {
  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!db.users[i].active) continue;

    if (strcmp(db.users[i].pin, pin) == 0) {
      return true;
    }
  }

  return false;
}

bool dbUidAuthorized(const char *uid) {
  if (db.adminCanOpenDoor && dbIsAdminUid(uid)) {
    return true;
  }

  return dbUserUidExists(uid);
}

bool dbPinAuthorized(const char *pin) {
  // Admin PIN can also open the door.
  if (dbIsAdminPin(pin)) {
    return true;
  }

  return dbUserPinExists(pin);
}

bool dbAddUser(const char *uid, const char *pin) {
  if (dbIsAdminUid(uid)) {
    Serial.println("[ESP2] Admin card already has access. Not adding to user list.");
    return false;
  }

  if (dbUserUidExists(uid)) {
    return false;
  }

  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!db.users[i].active) {
      safeCopy(db.users[i].uid, uid, sizeof(db.users[i].uid));
      safeCopy(db.users[i].pin, pin, sizeof(db.users[i].pin));
      db.users[i].active = true;
      return true;
    }
  }

  return false;
}

bool dbDeleteUserByUid(const char *uid) {
  if (dbIsAdminUid(uid)) {
    Serial.println("[ESP2] Admin card cannot be deleted.");
    return false;
  }

  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!db.users[i].active) continue;

    if (strcmp(db.users[i].uid, uid) == 0) {
      memset(&db.users[i], 0, sizeof(db.users[i]));
      return true;
    }
  }

  return false;
}

uint8_t dbCountUsers() {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (db.users[i].active) {
      count++;
    }
  }

  return count;
}

void dbPrintDebug() {
  Serial.println("[ESP2] ===== RAM DATABASE =====");
  Serial.print("[ESP2] setupDone: ");
  Serial.println(db.setupDone ? "YES" : "NO");
  Serial.print("[ESP2] adminUid: ");
  Serial.println(db.adminUid);
  Serial.print("[ESP2] user count: ");
  Serial.println(dbCountUsers());

  for (uint8_t i = 0; i < MAX_USERS; i++) {
    if (!db.users[i].active) continue;

    Serial.print("[ESP2] user[");
    Serial.print(i);
    Serial.print("] UID=");
    Serial.print(db.users[i].uid);
    Serial.print(" PIN=");
    Serial.println(db.users[i].pin);
  }
}

// ======================================================
// ESP-NOW SEND
// ======================================================
void sendCommandToEsp1(MsgType type) {
  EspNowPacket packet = {};
  packet.type = type;
  packet.counter = nextCounter();

  if (xQueueSend(qEspNowTx, &packet, 0) != pdTRUE) {
    Serial.println("[ESP2] WARN: qEspNowTx full");
  }
}

// ======================================================
// LCD SCREEN HELPERS
// ======================================================
void showIdle() {
  state = STATE_IDLE;
  clearInput();
  stopPinTimer();
  stopAdminTimer();

  postLcd("SMART DOOR", "SCAN/PIN A=ADM");
}

void showAdminMenu() {
  state = STATE_ADMIN_MENU;
  clearInput();
  restartAdminTimer();

  postLcd("1ADD 2DEL", "3INFO 4EXIT");
}

void showPinInputScreen(const char *title) {
  char mask[17];
  makePinMask(mask, sizeof(mask));

  char line2[17];
  snprintf(line2, sizeof(line2), "PIN:%s", mask);

  postLcd(title, line2);
}

// ======================================================
// TIMER CALLBACKS
// ======================================================
void onPinInputTimeout(TimerHandle_t xTimer) {
  postUiEvent(EV_PIN_TIMEOUT);
}

void onAdminTimeout(TimerHandle_t xTimer) {
  postUiEvent(EV_ADMIN_TIMEOUT);
}

void onLockoutEnd(TimerHandle_t xTimer) {
  postUiEvent(EV_LOCKOUT_END);
}

void onUiReturn(TimerHandle_t xTimer) {
  postUiEvent(EV_UI_RETURN);
}

// ======================================================
// ESP-NOW CALLBACKS
// ======================================================
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  // Keep quiet to avoid Serial spam.
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
#else
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len != sizeof(EspNowPacket)) return;

  UiEvent ev = {};
  ev.type = EV_PACKET;
  memcpy(&ev.packet, incomingData, sizeof(EspNowPacket));

  xQueueSend(qUiEvent, &ev, 0);
}

// ======================================================
// TASKS
// ======================================================
void EspNowTxTask(void *pv) {
  EspNowPacket packet;

  while (1) {
    if (xQueueReceive(qEspNowTx, &packet, portMAX_DELAY) == pdTRUE) {
      esp_err_t result = esp_now_send(PEER_MAC, (uint8_t *)&packet, sizeof(packet));

      if (result != ESP_OK) {
        Serial.print("[ESP2] ESP-NOW send error: ");
        Serial.println(result);
      }
    }
  }
}

void LcdTask(void *pv) {
  LcdMessage msg;
  char lastLine1[17] = "";
  char lastLine2[17] = "";

  while (1) {
    if (xQueueReceive(qLcd, &msg, portMAX_DELAY) == pdTRUE) {
      if (strcmp(msg.line1, lastLine1) == 0 && strcmp(msg.line2, lastLine2) == 0) {
        continue;
      }

      xSemaphoreTake(lcdMutex, portMAX_DELAY);

      lcd.setCursor(0, 0);
      lcd.print("                ");
      lcd.setCursor(0, 0);
      lcd.print(msg.line1);

      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print(msg.line2);

      xSemaphoreGive(lcdMutex);

      safeCopy(lastLine1, msg.line1, sizeof(lastLine1));
      safeCopy(lastLine2, msg.line2, sizeof(lastLine2));
    }
  }
}

void KeypadTask(void *pv) {
  while (1) {
    char key = keypad.getKey();

    if (key) {
      UiEvent ev = {};
      ev.type = EV_KEY;
      ev.key = key;
      xQueueSend(qUiEvent, &ev, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ======================================================
// STATE MACHINE HANDLERS
// ======================================================
void handleAccessDenied() {
  failedAttempts++;

  sendCommandToEsp1(MSG_CMD_DENY);

  if (failedAttempts >= 3) {
    state = STATE_LOCKED_OUT;
    postLcd("TOO MANY FAIL", "LOCKED 30 SEC");
    xTimerStart(lockoutTimer, 0);
    return;
  }

  state = STATE_ACCESS_DENIED;
  postLcd("ACCESS DENIED", "TRY AGAIN");
  startUiReturnTimer(1500);
}

void handleAccessGranted(const char *line1) {
  failedAttempts = 0;

  sendCommandToEsp1(MSG_CMD_OPEN_DOOR);

  state = STATE_ACCESS_GRANTED;
  postLcd(line1, "DOOR OPENING");
}

void handleKeyEvent(char key) {
  if (state == STATE_FIRE_ALARM) {
    if (key == 'D') {
      postLcd("FIRE MODE", "RESET DEVICE");
    }
    return;
  }

  if (state == STATE_LOCKED_OUT) {
    return;
  }

  switch (state) {
    case STATE_FIRST_SET_ADMIN_PIN:
      if (keyIsDigit(key)) {
        appendDigit(key);
        showPinInputScreen("SET ADMIN PIN");
        restartPinTimer();
      } else if (key == '*') {
        backspaceInput();
        showPinInputScreen("SET ADMIN PIN");
      } else if (key == '#') {
        if (inputLen >= PIN_MIN_LEN) {
          safeCopy(pendingAdminPin, inputBuf, sizeof(pendingAdminPin));
          clearInput();

          state = STATE_FIRST_SCAN_ADMIN_CARD;
          stopPinTimer();
          postLcd("SCAN ADMIN", "CARD NOW");
        } else {
          postLcd("PIN TOO SHORT", "MIN 4 DIGITS");
          startUiReturnTimer(1300);
        }
      }
      break;

    case STATE_IDLE:
      if (keyIsDigit(key)) {
        clearInput();
        appendDigit(key);

        state = STATE_ENTER_USER_PIN;
        showPinInputScreen("ENTER PIN");
        restartPinTimer();
      } else if (key == 'A') {
        clearInput();

        state = STATE_ADMIN_LOGIN;
        showPinInputScreen("ADMIN LOGIN");
        restartPinTimer();
      }
      break;

    case STATE_ENTER_USER_PIN:
      if (keyIsDigit(key)) {
        appendDigit(key);
        showPinInputScreen("ENTER PIN");
        restartPinTimer();
      } else if (key == '*') {
        backspaceInput();
        showPinInputScreen("ENTER PIN");
        restartPinTimer();
      } else if (key == 'B') {
        showIdle();
      } else if (key == '#') {
        stopPinTimer();

        if (dbPinAuthorized(inputBuf)) {
          handleAccessGranted("PIN ACCEPTED");
        } else {
          handleAccessDenied();
        }

        clearInput();
      }
      break;

    case STATE_ADMIN_LOGIN:
      if (keyIsDigit(key)) {
        appendDigit(key);
        showPinInputScreen("ADMIN LOGIN");
        restartPinTimer();
      } else if (key == '*') {
        backspaceInput();
        showPinInputScreen("ADMIN LOGIN");
        restartPinTimer();
      } else if (key == 'B') {
        showIdle();
      } else if (key == '#') {
        stopPinTimer();

        if (dbIsAdminPin(inputBuf)) {
          failedAttempts = 0;
          showAdminMenu();
        } else {
          handleAccessDenied();
        }

        clearInput();
      }
      break;

    case STATE_ADMIN_MENU:
      restartAdminTimer();

      if (key == '1') {
        state = STATE_ADD_USER_WAIT_CARD;
        pendingUid[0] = '\0';
        postLcd("ADD USER", "SCAN CARD");
      } else if (key == '2') {
        state = STATE_DELETE_USER_WAIT_CARD;
        postLcd("DELETE USER", "SCAN CARD");
      } else if (key == '3') {
        char line2[17];
        snprintf(line2, sizeof(line2), "USERS:%u/%u", dbCountUsers(), MAX_USERS);
        postLcd("DATABASE INFO", line2);
      } else if (key == '4' || key == 'B') {
        showIdle();
      }
      break;

    case STATE_ADD_USER_WAIT_CARD:
      if (key == 'B' || key == '*') {
        showAdminMenu();
      }
      break;

    case STATE_ADD_USER_WAIT_PIN:
      if (keyIsDigit(key)) {
        appendDigit(key);
        showPinInputScreen("NEW USER PIN");
        restartPinTimer();
      } else if (key == '*') {
        backspaceInput();
        showPinInputScreen("NEW USER PIN");
        restartPinTimer();
      } else if (key == 'B') {
        showAdminMenu();
      } else if (key == '#') {
        stopPinTimer();

        if (inputLen < PIN_MIN_LEN) {
          postLcd("PIN TOO SHORT", "MIN 4 DIGITS");
          restartPinTimer();
        } else {
          bool ok = dbAddUser(pendingUid, inputBuf);

          if (ok) {
            postLcd("USER SAVED", pendingUid);
          } else {
            postLcd("SAVE FAILED", "DUP/FULL");
          }

          clearInput();
          pendingUid[0] = '\0';
          state = STATE_ADMIN_MENU;
          restartAdminTimer();
        }
      }
      break;

    case STATE_DELETE_USER_WAIT_CARD:
      if (key == 'B' || key == '*') {
        showAdminMenu();
      }
      break;

    default:
      break;
  }
}

void handlePacketEvent(const EspNowPacket &packet) {
  switch (packet.type) {
    case MSG_TEMP_REPORT:
      Serial.print("[ESP2] Temperature from ESP1: ");
      Serial.print(packet.temperatureC, 1);
      Serial.print(" C | Humidity: ");
      Serial.print(packet.humidity, 1);
      Serial.println(" %");
      break;

    case MSG_FIRE_ALARM:
      Serial.println("[ESP2] FIRE ALARM from ESP1");
      state = STATE_FIRE_ALARM;
      postLcd("FIRE ALERT!", "DOOR OPEN");
      break;

    case MSG_DOOR_OPENED:
      Serial.println("[ESP2] Door opened");
      if (state != STATE_ADMIN_MENU &&
          state != STATE_ADD_USER_WAIT_CARD &&
          state != STATE_ADD_USER_WAIT_PIN &&
          state != STATE_DELETE_USER_WAIT_CARD &&
          state != STATE_FIRE_ALARM) {
        postLcd("DOOR STATUS", "OPEN");
      }
      break;

    case MSG_DOOR_CLOSED:
      Serial.println("[ESP2] Door closed");
      if (state == STATE_ACCESS_GRANTED) {
        showIdle();
      } else if (state == STATE_IDLE) {
        postLcd("DOOR STATUS", "CLOSED");
        startUiReturnTimer(1200);
      }
      break;

    case MSG_CARD_SCANNED:
      Serial.print("[ESP2] RFID UID from ESP1: ");
      Serial.println(packet.uid);

      if (state == STATE_FIRST_SCAN_ADMIN_CARD) {
        safeCopy(db.adminPin, pendingAdminPin, sizeof(db.adminPin));
        safeCopy(db.adminUid, packet.uid, sizeof(db.adminUid));
        db.adminCanOpenDoor = true;
        dbSetInitialized(true);

        pendingAdminPin[0] = '\0';
        clearInput();

        Serial.print("[ESP2] Admin card saved in RAM: ");
        Serial.println(db.adminUid);

        postLcd("SETUP DONE", "ADMIN SAVED");
        startUiReturnTimer(1800);
        state = STATE_ACCESS_GRANTED;
        return;
      }

      if (state == STATE_ADD_USER_WAIT_CARD) {
        if (dbIsAdminUid(packet.uid)) {
          postLcd("ADMIN CARD", "ALREADY VALID");
          state = STATE_ADMIN_MENU;
          restartAdminTimer();
        } else if (dbUserUidExists(packet.uid)) {
          postLcd("UID EXISTS", "NOT SAVED");
          state = STATE_ADMIN_MENU;
          restartAdminTimer();
        } else {
          safeCopy(pendingUid, packet.uid, sizeof(pendingUid));
          clearInput();

          state = STATE_ADD_USER_WAIT_PIN;
          postLcd("CARD CAPTURED", "SET USER PIN");
          restartPinTimer();
        }
        return;
      }

      if (state == STATE_DELETE_USER_WAIT_CARD) {
        if (dbIsAdminUid(packet.uid)) {
          postLcd("ADMIN CARD", "CANNOT DELETE");
        } else {
          bool deleted = dbDeleteUserByUid(packet.uid);

          if (deleted) {
            postLcd("USER DELETED", packet.uid);
          } else {
            postLcd("DELETE FAILED", "NOT FOUND");
          }
        }

        state = STATE_ADMIN_MENU;
        restartAdminTimer();
        return;
      }

      if (state == STATE_LOCKED_OUT) {
        sendCommandToEsp1(MSG_CMD_DENY);
        return;
      }

      if (dbIsAdminUid(packet.uid)) {
        handleAccessGranted("ADMIN ACCESS");
      } else if (dbUserUidExists(packet.uid)) {
        handleAccessGranted("CARD ACCEPTED");
      } else {
        handleAccessDenied();
      }
      break;

    default:
      break;
  }
}

void handleTimerEvent(UiEventType type) {
  switch (type) {
    case EV_PIN_TIMEOUT:
      clearInput();

      if (state == STATE_FIRST_SET_ADMIN_PIN) {
        postLcd("SET ADMIN PIN", "TIMEOUT");
        startUiReturnTimer(1200);
      } else if (state == STATE_ADD_USER_WAIT_PIN) {
        postLcd("ADD USER", "PIN TIMEOUT");
        state = STATE_ADMIN_MENU;
        restartAdminTimer();
      } else {
        showIdle();
      }
      break;

    case EV_ADMIN_TIMEOUT:
      if (state == STATE_ADMIN_MENU ||
          state == STATE_ADD_USER_WAIT_CARD ||
          state == STATE_ADD_USER_WAIT_PIN ||
          state == STATE_DELETE_USER_WAIT_CARD) {
        showIdle();
      }
      break;

    case EV_LOCKOUT_END:
      failedAttempts = 0;
      showIdle();
      break;

    case EV_UI_RETURN:
      if (!dbInitialized()) {
        state = STATE_FIRST_SET_ADMIN_PIN;
        clearInput();
        postLcd("FIRST SETUP", "SET ADMIN PIN");
      } else if (state == STATE_ACCESS_GRANTED ||
                 state == STATE_ACCESS_DENIED ||
                 state == STATE_IDLE) {
        showIdle();
      }
      break;

    default:
      break;
  }
}

void ManagerTask(void *pv) {
  if (!dbInitialized()) {
    state = STATE_FIRST_SET_ADMIN_PIN;
    clearInput();
    postLcd("FIRST SETUP", "SET ADMIN PIN");
  } else {
    showIdle();
  }

  UiEvent ev;

  while (1) {
    if (xQueueReceive(qUiEvent, &ev, portMAX_DELAY) == pdTRUE) {
      switch (ev.type) {
        case EV_KEY:
          handleKeyEvent(ev.key);
          break;

        case EV_PACKET:
          handlePacketEvent(ev.packet);
          break;

        case EV_PIN_TIMEOUT:
        case EV_ADMIN_TIMEOUT:
        case EV_LOCKOUT_END:
        case EV_UI_RETURN:
          handleTimerEvent(ev.type);
          break;

        default:
          break;
      }
    }
  }
}

void HeartbeatTask(void *pv) {
  while (1) {
    Serial.print("[ESP2] HEARTBEAT state=");
    Serial.print((int)state);
    Serial.print(" setupDone=");
    Serial.print(db.setupDone ? "YES" : "NO");
    Serial.print(" users=");
    Serial.println(dbCountUsers());
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ======================================================
// INIT
// ======================================================
void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[ESP2] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP2] ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, PEER_MAC, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (!esp_now_is_peer_exist(PEER_MAC)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP2] Failed to add ESP-NOW peer");
      return;
    }
  }

  Serial.println("[ESP2] ESP-NOW ready");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("========== ESP2 MANAGER/UI NODE RAM-ONLY ==========");

  dbResetRuntime();
  Serial.println("[ESP2] RAM database cleared. First setup required.");

  qUiEvent = xQueueCreate(16, sizeof(UiEvent));
  qLcd = xQueueCreate(8, sizeof(LcdMessage));
  qEspNowTx = xQueueCreate(12, sizeof(EspNowPacket));

  lcdMutex = xSemaphoreCreateMutex();

  pinInputTimeoutTimer = xTimerCreate(
    "PinTimeout",
    pdMS_TO_TICKS(10000),
    pdFALSE,
    NULL,
    onPinInputTimeout
  );

  adminTimeoutTimer = xTimerCreate(
    "AdminTimeout",
    pdMS_TO_TICKS(30000),
    pdFALSE,
    NULL,
    onAdminTimeout
  );

  lockoutTimer = xTimerCreate(
    "Lockout",
    pdMS_TO_TICKS(30000),
    pdFALSE,
    NULL,
    onLockoutEnd
  );

  uiReturnTimer = xTimerCreate(
    "UiReturn",
    pdMS_TO_TICKS(1500),
    pdFALSE,
    NULL,
    onUiReturn
  );

  Wire.begin(I2C_SDA, I2C_SCL);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP2 UI NODE");
  lcd.setCursor(0, 1);
  lcd.print("BOOTING...");

  initEspNow();

  xTaskCreate(EspNowTxTask, "EspNowTx", 4096, NULL, 3, NULL);
  xTaskCreate(LcdTask, "LCD", 4096, NULL, 2, NULL);
  xTaskCreate(KeypadTask, "Keypad", 4096, NULL, 2, NULL);
  xTaskCreate(ManagerTask, "Manager", 8192, NULL, 3, NULL);
  xTaskCreate(HeartbeatTask, "Heartbeat", 2048, NULL, 1, NULL);

  Serial.println("[ESP2] Manager/UI node ready.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}