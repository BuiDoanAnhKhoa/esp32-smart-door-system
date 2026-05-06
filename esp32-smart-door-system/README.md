# ESP32 Smart Door Access Control System

## 1. Giới thiệu

Đây là hệ thống điều khiển cửa thông minh sử dụng 2 board ESP32 giao tiếp với nhau thông qua ESP-NOW. Hệ thống hỗ trợ xác thực bằng thẻ RFID, mã PIN qua keypad, hiển thị trạng thái trên LCD, điều khiển servo đóng/mở cửa, cảnh báo bằng buzzer và giám sát nhiệt độ bằng cảm biến DHT11.

Dự án được chia thành 2 node chính:

- **ESP1_RFID_Door**: xử lý RFID, servo, buzzer, DHT11 và trạng thái cửa.
- **ESP2_Keypad_LCD**: xử lý keypad, LCD, quản lý người dùng, mã PIN và quyền truy cập.

## 2. Chức năng chính

### ESP1 - RFID, Servo, DHT11, Buzzer 

- Đọc UID từ thẻ RFID MFRC522.
- Gửi UID sang ESP2 bằng ESP-NOW để kiểm tra quyền truy cập.
- Nhận lệnh mở/đóng cửa từ ESP2.
- Điều khiển servo để mở hoặc đóng cửa.
- Tự động đóng cửa sau một khoảng thời gian.
- Đọc nhiệt độ và độ ẩm từ cảm biến DHT11.
- Phát hiện quá nhiệt và kích hoạt chế độ cảnh báo cháy.
- Điều khiển buzzer theo các trạng thái: thành công, từ chối, cảnh báo cháy.
- Hỗ trợ nút khẩn cấp để mở cửa trong trường hợp cần thiết (giả lập Fire Alarm).

### ESP2 - Keypad, LCD, Quản lý người dùng

- Hiển thị trạng thái hệ thống trên LCD I2C 16x2.
- Nhập mã PIN thông qua keypad 4x4.
- Thiết lập mã PIN admin lần đầu.
- Quét thẻ admin lần đầu để hoàn tất cấu hình hệ thống.
- Cho phép admin thêm người dùng mới.
- Cho phép admin xóa người dùng bằng UID thẻ.
- Kiểm tra quyền truy cập bằng UID hoặc mã PIN.
- Gửi lệnh mở cửa, đóng cửa hoặc từ chối truy cập đến ESP1.
- Cơ chế khóa tạm thời khi nhập sai nhiều lần.
- Hiển thị nhiệt độ, độ ẩm (thông qua Serial Monitor) và trạng thái cửa.

## 3. Kiến trúc hệ thống

Hai ESP32 giao tiếp với nhau bằng **ESP-NOW**.

### Sơ đồ tổng quát

```text
ESP1_RFID_Door  <-------------------->  ESP2_Keypad_LCD
                   ESP-NOW wireless
```

### Chức năng từng ESP

| ESP1_RFID_Door | ESP2_Keypad_LCD |
|---|---|
| RFID RC522 | Keypad 4x4 |
| Servo Motor | LCD I2C 16x2 |
| DHT11 Sensor | User Management |
| Buzzer | PIN Authentication |
| Emergency Button | Access Control Logic |

## 4. Required Libraries
Install these libraries in Arduino IDE:

1. esp32 by Espressif Systems
2. MFRC522
3. ESP32Servo
4. DHT sensor library
5. Adafruit Unified Sensor
6. LiquidCrystal I2C
7. Keypad
