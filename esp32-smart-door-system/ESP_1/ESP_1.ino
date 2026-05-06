#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <DHT.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

// ======================================================
// SYSTEM CONFIG
// ======================================================
#define SERIAL_BAUD 115200
#define WIFI_CHANNEL 1

// Broadcast by default.
// After stable testing, replace this with ESP2 MAC for unicast.
uint8_t PEER_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define UID_TEXT_LEN 32

// ======================================================
// ESP-NOW PROTOCOL
// Must match ESP2 exactly
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
// PINS
// ======================================================
#define RFID_SCK   4
#define RFID_MISO  5
#define RFID_MOSI  6
#define RFID_SS    7
#define RFID_RST   10

#define DHT_PIN    0
#define DHTTYPE    DHT11

#define SERVO_PIN  2
#define BUZZER_PIN 3

// Optional emergency button.
// Button wiring: GPIO1 ---- button ---- GND
#define USE_EMERGENCY_BUTTON 1
#define EMERGENCY_BUTTON_PIN 1

// ======================================================
// DOOR CONFIG
// ======================================================
#define SERVO_CLOSED_ANGLE 20
#define SERVO_OPEN_ANGLE   110

#define DOOR_OPEN_TIME_MS 3000
#define DHT_PERIOD_MS 2000
#define OVERHEAT_TEMP_C 35

const bool BUZZER_ACTIVE_LOW = false;

// ======================================================
// RTOS OBJECTS
// ======================================================
QueueHandle_t qEspNowTx;
QueueHandle_t qEspNowRx;
QueueHandle_t qServoCmd;
QueueHandle_t qBuzzerCmd;

TimerHandle_t doorAutoCloseTimer;

EventGroupHandle_t systemEvents;
SemaphoreHandle_t stateMutex;

#if USE_EMERGENCY_BUTTON
SemaphoreHandle_t emergencySem;
#endif

#define BIT_DOOR_OPEN   (1 << 0)
#define BIT_FIRE_ALARM  (1 << 1)

// ======================================================
// HARDWARE OBJECTS
// ======================================================
MFRC522 rfid(RFID_SS, RFID_RST);
Servo doorServo;
DHT dht(DHT_PIN, DHTTYPE);

// ======================================================
// STATE
// ======================================================
float currentTempC = NAN;
float currentHumidity = NAN;

uint32_t packetCounter = 0;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

// ======================================================
// COMMAND TYPES
// ======================================================
enum ServoCmdType : uint8_t {
  SERVO_CMD_OPEN = 1,
  SERVO_CMD_CLOSE,
  SERVO_CMD_FIRE_OPEN
};

typedef struct {
  ServoCmdType type;
} ServoCommand;

enum BuzzerPattern : uint8_t {
  BUZZ_NONE = 0,
  BUZZ_SUCCESS,
  BUZZ_DENIED,
  BUZZ_FIRE
};

typedef struct {
  BuzzerPattern pattern;
} BuzzerCommand;

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

void uidToString(MFRC522::Uid *uid, char *out, size_t outSize) {
  out[0] = '\0';

  for (byte i = 0; i < uid->size; i++) {
    char part[4];

    if (i < uid->size - 1) {
      snprintf(part, sizeof(part), "%02X:", uid->uidByte[i]);
    } else {
      snprintf(part, sizeof(part), "%02X", uid->uidByte[i]);
    }

    strncat(out, part, outSize - strlen(out) - 1);
  }
}

void enqueueEspNowPacket(MsgType type, const char *uid = "") {
  EspNowPacket packet = {};
  packet.type = type;
  safeCopy(packet.uid, uid, sizeof(packet.uid));

  xSemaphoreTake(stateMutex, portMAX_DELAY);
  packet.temperatureC = currentTempC;
  packet.humidity = currentHumidity;
  xSemaphoreGive(stateMutex);

  EventBits_t bits = xEventGroupGetBits(systemEvents);
  packet.doorOpen = bits & BIT_DOOR_OPEN;
  packet.fireAlarm = bits & BIT_FIRE_ALARM;
  packet.counter = nextCounter();

  xQueueSend(qEspNowTx, &packet, 0);
}

void queueServoCommand(ServoCmdType type) {
  ServoCommand cmd = {};
  cmd.type = type;
  xQueueSend(qServoCmd, &cmd, 0);
}

void queueBuzzerCommand(BuzzerPattern pattern) {
  BuzzerCommand cmd = {};
  cmd.pattern = pattern;
  xQueueSend(qBuzzerCmd, &cmd, 0);
}

// ======================================================
// BUZZER
// ======================================================
void buzzerWrite(bool on) {
  if (BUZZER_ACTIVE_LOW) {
    digitalWrite(BUZZER_PIN, on ? LOW : HIGH);
  } else {
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  }
}

void beepOnce(uint32_t onMs, uint32_t offMs) {
  buzzerWrite(true);
  vTaskDelay(pdMS_TO_TICKS(onMs));
  buzzerWrite(false);
  vTaskDelay(pdMS_TO_TICKS(offMs));
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

  EspNowPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  xQueueSend(qEspNowRx, &packet, 0);
}

// ======================================================
// SOFTWARE TIMER CALLBACK
// ======================================================
void onDoorAutoCloseTimer(TimerHandle_t xTimer) {
  ServoCommand cmd = {};
  cmd.type = SERVO_CMD_CLOSE;
  xQueueSend(qServoCmd, &cmd, 0);
}

// ======================================================
// OPTIONAL INTERRUPT
// ======================================================
#if USE_EMERGENCY_BUTTON
void IRAM_ATTR emergencyButtonIsr() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(emergencySem, &higherPriorityTaskWoken);

  if (higherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}
#endif

// ======================================================
// TASKS
// ======================================================
void EspNowTxTask(void *pv) {
  EspNowPacket packet;

  while (1) {
    if (xQueueReceive(qEspNowTx, &packet, portMAX_DELAY) == pdTRUE) {
      esp_err_t result = esp_now_send(PEER_MAC, (uint8_t *)&packet, sizeof(packet));

      if (result != ESP_OK) {
        Serial.print("[ESP1] ESP-NOW send error: ");
        Serial.println(result);
      }
    }
  }
}

void EspNowRxTask(void *pv) {
  EspNowPacket packet;

  while (1) {
    if (xQueueReceive(qEspNowRx, &packet, portMAX_DELAY) == pdTRUE) {
      switch (packet.type) {
        case MSG_CMD_OPEN_DOOR:
          Serial.println("[ESP1] Command received: OPEN_DOOR");
          queueBuzzerCommand(BUZZ_SUCCESS);
          queueServoCommand(SERVO_CMD_OPEN);
          break;

        case MSG_CMD_CLOSE_DOOR:
          Serial.println("[ESP1] Command received: CLOSE_DOOR");
          queueServoCommand(SERVO_CMD_CLOSE);
          break;

        case MSG_CMD_DENY:
          Serial.println("[ESP1] Command received: DENY");
          queueBuzzerCommand(BUZZ_DENIED);
          break;

        default:
          break;
      }
    }
  }
}

void RfidTask(void *pv) {
  char lastUid[UID_TEXT_LEN] = "";
  uint32_t lastSeenMs = 0;
  const uint32_t cardRemoveTimeoutMs = 900;

  while (1) {
    if (!rfid.PICC_IsNewCardPresent()) {
      if (millis() - lastSeenMs > cardRemoveTimeoutMs) {
        lastUid[0] = '\0';
      }

      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }

    if (!rfid.PICC_ReadCardSerial()) {
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }

    lastSeenMs = millis();

    char uidText[UID_TEXT_LEN];
    uidToString(&rfid.uid, uidText, sizeof(uidText));

    if (strcmp(uidText, lastUid) != 0) {
      safeCopy(lastUid, uidText, sizeof(lastUid));

      Serial.println();
      Serial.println("[ESP1] RFID card scanned");
      Serial.print("[ESP1] UID: ");
      Serial.println(uidText);

      enqueueEspNowPacket(MSG_CARD_SCANNED, uidText);
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

void DhtTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("[ESP1] DHT11 read failed");
    } else {
      xSemaphoreTake(stateMutex, portMAX_DELAY);
      currentTempC = t;
      currentHumidity = h;
      xSemaphoreGive(stateMutex);

      Serial.print("[ESP1] Temperature: ");
      Serial.print(t, 1);
      Serial.print(" C | Humidity: ");
      Serial.print(h, 1);
      Serial.println(" %");

      enqueueEspNowPacket(MSG_TEMP_REPORT);

      EventBits_t bits = xEventGroupGetBits(systemEvents);

      if (!(bits & BIT_FIRE_ALARM) && t >= OVERHEAT_TEMP_C) {
        Serial.println("[ESP1] OVERHEAT DETECTED. Emergency opening door.");

        xEventGroupSetBits(systemEvents, BIT_FIRE_ALARM);

        queueServoCommand(SERVO_CMD_FIRE_OPEN);
        queueBuzzerCommand(BUZZ_FIRE);
        enqueueEspNowPacket(MSG_FIRE_ALARM);
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DHT_PERIOD_MS));
  }
}

void ServoTask(void *pv) {
  ServoCommand cmd;

  while (1) {
    if (xQueueReceive(qServoCmd, &cmd, portMAX_DELAY) == pdTRUE) {
      EventBits_t bits = xEventGroupGetBits(systemEvents);
      bool fireAlarm = bits & BIT_FIRE_ALARM;

      switch (cmd.type) {
        case SERVO_CMD_OPEN:
          if (fireAlarm) {
            Serial.println("[ESP1] Fire alarm active. Door already forced OPEN.");
          }

          Serial.println("[ESP1] Door OPEN");
          doorServo.write(SERVO_OPEN_ANGLE);
          xEventGroupSetBits(systemEvents, BIT_DOOR_OPEN);
          enqueueEspNowPacket(MSG_DOOR_OPENED);

          if (!fireAlarm) {
            xTimerStop(doorAutoCloseTimer, 0);
            xTimerStart(doorAutoCloseTimer, 0);
          }
          break;

        case SERVO_CMD_CLOSE:
          if (fireAlarm) {
            Serial.println("[ESP1] Fire alarm active. Door close blocked.");
            break;
          }

          Serial.println("[ESP1] Door CLOSED");
          doorServo.write(SERVO_CLOSED_ANGLE);
          xEventGroupClearBits(systemEvents, BIT_DOOR_OPEN);
          enqueueEspNowPacket(MSG_DOOR_CLOSED);
          break;

        case SERVO_CMD_FIRE_OPEN:
          Serial.println("[ESP1] FIRE OPEN. Door forced OPEN.");
          xTimerStop(doorAutoCloseTimer, 0);
          doorServo.write(SERVO_OPEN_ANGLE);
          xEventGroupSetBits(systemEvents, BIT_DOOR_OPEN | BIT_FIRE_ALARM);
          enqueueEspNowPacket(MSG_FIRE_ALARM);
          break;
      }
    }
  }
}

void BuzzerTask(void *pv) {
  BuzzerCommand cmd;

  while (1) {
    if (xQueueReceive(qBuzzerCmd, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.pattern) {
        case BUZZ_SUCCESS:
          beepOnce(80, 60);
          beepOnce(80, 40);
          break;

        case BUZZ_DENIED:
          beepOnce(450, 80);
          break;

        case BUZZ_FIRE:
          for (int i = 0; i < 8; i++) {
            beepOnce(160, 120);
          }
          break;

        default:
          break;
      }

      buzzerWrite(false);
    }
  }
}

#if USE_EMERGENCY_BUTTON
void EmergencyTask(void *pv) {
  uint32_t lastTriggerMs = 0;

  while (1) {
    if (xSemaphoreTake(emergencySem, portMAX_DELAY) == pdTRUE) {
      uint32_t now = millis();

      if (now - lastTriggerMs < 400) {
        continue;
      }

      lastTriggerMs = now;

      Serial.println("[ESP1] Emergency button interrupt detected.");
      xEventGroupSetBits(systemEvents, BIT_FIRE_ALARM);

      queueServoCommand(SERVO_CMD_FIRE_OPEN);
      queueBuzzerCommand(BUZZ_FIRE);
      enqueueEspNowPacket(MSG_FIRE_ALARM);
    }
  }
}
#endif

// ======================================================
// INIT
// ======================================================
void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[ESP1] MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP1] ESP-NOW init failed");
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
      Serial.println("[ESP1] Failed to add ESP-NOW peer");
      return;
    }
  }

  Serial.println("[ESP1] ESP-NOW ready");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("========== ESP1 DOOR NODE ==========");

  qEspNowTx = xQueueCreate(12, sizeof(EspNowPacket));
  qEspNowRx = xQueueCreate(12, sizeof(EspNowPacket));
  qServoCmd = xQueueCreate(8, sizeof(ServoCommand));
  qBuzzerCmd = xQueueCreate(8, sizeof(BuzzerCommand));

  systemEvents = xEventGroupCreate();
  stateMutex = xSemaphoreCreateMutex();

#if USE_EMERGENCY_BUTTON
  emergencySem = xSemaphoreCreateBinary();
#endif

  doorAutoCloseTimer = xTimerCreate(
    "DoorCloseTimer",
    pdMS_TO_TICKS(DOOR_OPEN_TIME_MS),
    pdFALSE,
    NULL,
    onDoorAutoCloseTimer
  );

  pinMode(BUZZER_PIN, OUTPUT);
  buzzerWrite(false);

#if USE_EMERGENCY_BUTTON
  pinMode(EMERGENCY_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(EMERGENCY_BUTTON_PIN),
    emergencyButtonIsr,
    FALLING
  );
#endif

  dht.begin();

  doorServo.setPeriodHertz(50);
  doorServo.attach(SERVO_PIN, 600, 2300);
  doorServo.write(SERVO_CLOSED_ANGLE);
  xEventGroupClearBits(systemEvents, BIT_DOOR_OPEN | BIT_FIRE_ALARM);

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  rfid.PCD_Init();
  delay(100);

  Serial.print("[ESP1] RC522: ");
  rfid.PCD_DumpVersionToSerial();

  initEspNow();

  xTaskCreate(EspNowTxTask, "EspNowTx", 4096, NULL, 3, NULL);
  xTaskCreate(EspNowRxTask, "EspNowRx", 4096, NULL, 3, NULL);
  xTaskCreate(RfidTask, "RFID", 4096, NULL, 2, NULL);
  xTaskCreate(DhtTask, "DHT", 4096, NULL, 1, NULL);
  xTaskCreate(ServoTask, "Servo", 4096, NULL, 3, NULL);
  xTaskCreate(BuzzerTask, "Buzzer", 3072, NULL, 2, NULL);

#if USE_EMERGENCY_BUTTON
  xTaskCreate(EmergencyTask, "Emergency", 3072, NULL, 4, NULL);
#endif

  enqueueEspNowPacket(MSG_DOOR_CLOSED);

  Serial.println("[ESP1] Door node ready.");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}