# Self-Balancing-Robot-Two-wheel-ESP32-FreeRTOS-MQTT-Firebase-LQR-
A real-time self-balancing robot built on ESP32 dual-core processor, featuring LQR-based control with Kalman filtering, FreeRTOS multitasking architecture, and IoT cloud connectivity through MQTT and Firebase.

## Tính năng

- Điều khiển cân bằng real-time (3ms loop) với Kalman filter
- FreeRTOS dual-core: Core 1 (balance), Core 0 (MQTT/Firebase)
- Điều khiển từ xa qua MQTT (F/B/L/R/S)
- Telemetry real-time qua Firebase RTDB

## Phần cứng

| Linh kiện | Model |
|-----------|-------|
| MCU | ESP32-WROOM-32 |
| IMU | MPU6050 |
| Motor Driver | L298N |
| Encoder | Quadrature |

## Cài đặt

1. Cấu hình WiFi và cloud credentials trong `balancing_car_v5.ino`
2. Upload lên ESP32 bằng Arduino IDE

## Thông số điều khiển
```
K3 = 800   (góc nghiêng)
K4 = 1.5   (tốc độ góc)
Control loop: 3ms
Max recovery: ±15°
```

## MQTT Topics

| Topic | Mô tả |
|-------|-------|
| `.../telemetry` | Dữ liệu sensor (10Hz) |
| `.../command` | Lệnh điều khiển |
| `.../config` | Cấu hình K3, K4, offset |

## Cấu trúc project
```
├── balancing_car_v5.ino   # Firmware chính
├── Main.py                # GUI dashboard
└── README.md
```
