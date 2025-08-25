# Hệ thống giám sát ngập lụt ESP32

## 1. Giới thiệu
Dự án sử dụng ESP32 để giám sát nhiệt độ, độ ẩm không khí, mưa, ngập nước và độ ẩm đất. Hệ thống có thể điều khiển bơm nước tự động hoặc từ xa qua MQTT, đồng thời cảnh báo bằng buzzer và LED RGB.

## 2. Thư mục & file
- **hoanthien.ino**: Code hoàn chỉnh, kết nối MQTT, dashboard web.
- **test.ino**: Code kiểm tra cảm biến, test phần cứng.
- **index.html**: Dashboard web hiển thị dữ liệu realtime.
- **readme.docx**: Tài liệu hướng dẫn chi tiết (bản Word).
- **README.md**: (file này) Hướng dẫn nhanh.

## 3. Phần cứng cần thiết
- ESP32 DevKit
- Cảm biến DHT11 (nhiệt độ, độ ẩm không khí)
- Cảm biến mưa (Rain sensor)
- Cảm biến nước (Water sensor)
- Cảm biến độ ẩm đất (Soil moisture sensor)
- Relay, Buzzer, LED RGB
- Kết nối Internet qua WiFi

## 4. Hướng dẫn sử dụng

### 4.1. Kiểm tra cảm biến
1. Mở file `test.ino` bằng Arduino IDE.
2. Chọn board ESP32, nạp code vào ESP32.
3. Mở Serial Monitor (baud 115200) để xem giá trị cảm biến và trạng thái bơm, LED.

### 4.2. Chạy hệ thống hoàn chỉnh
1. Mở file `hoanthien.ino` bằng Arduino IDE.
2. Sửa thông tin WiFi nếu cần (`WIFI_SSID`, `WIFI_PASS`).
3. Nạp code vào ESP32.
4. Đảm bảo các cảm biến được cấp nguồn 3.3V (nếu dùng 5V cần chia áp).
5. Khi ESP32 hoạt động, dữ liệu sẽ được gửi lên MQTT HiveMQ Cloud.

### 4.3. Xem dashboard web
- Mở file `index.html` trên trình duyệt máy tính (cần Internet).
- Dashboard sẽ tự động nhận dữ liệu từ ESP32 qua MQTT và hiển thị realtime.

## 5. Chức năng chính
- Đọc và hiển thị nhiệt độ, độ ẩm, mưa, nước, độ ẩm đất.
- Phân loại trạng thái: khô ráo, ẩm nhẹ, mưa nhỏ, mưa lớn, ngập thấp, ngập cao.
- Điều khiển bơm tự động hoặc thủ công từ dashboard.
- Cảnh báo bằng buzzer và LED RGB.

## 6. Lưu ý
- Nếu không kết nối được MQTT, kiểm tra lại WiFi và tài khoản HiveMQ.
- Đảm bảo dây nối cảm biến đúng chân như trong code.
- Có thể hiệu chỉnh ngưỡng cảm biến trong code cho phù hợp thực tế.

---

**Tác giả:**  
- Thuan  
- Liên hệ hỗ trợ: qua dashboard