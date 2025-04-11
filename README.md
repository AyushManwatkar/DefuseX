# DefuseX

1. Read real-time accelerometer and gyroscope data
2. Transmit sensor values over Serial
3. Control servo motors based on movement/gesture input
   
---

## 📦 Features

- 📈 Real-time motion data from MPU6050 (Accel + Gyro)
- 🔄 Serial communication for data logging or control feedback
- ⚙️ Servo motor control based on sensor inputs

---

## 🧠 Components Used

| Component           | Purpose                       |
|--------------------|-------------------------------|
| ESP32 WROOM Devkit | Main microcontroller          |
| MPU6050            | Accelerometer + Gyroscope     |
| Servo Motors       | Controlled via ESP32 PWM pins |

---

## 📁 File Structure
- Main.cpp - read and process all inputs from mpu6050 and give output to motors
- platformio.ini - Configure ESP32
