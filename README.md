# Xiaozhi ESP32-S3 Custom Robot Firmware 🤖

Custom firmware fork based on [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32), adapted by **Sol Digital Solutions (EPamar)** for an interactive desktop companion robot built on **ESP32-S3**.

---

### ✨ Custom Features & Modifications

* **Dedicated Robot Motion Controller (`robot_controller`):** Custom servo choreography, expressive movements, and physical action triggers synchronized with voice interaction states.
* **Custom Board Integration:** Adapted around the `xingzhi-cube-1.54tft-wifi` board configuration with a 1.54" TFT display, onboard audio I/O, and battery power management (`power_manager`).
* **Live Internet Radio Streaming:** Compatible with [xiaozhi-radio-server](https://github.com/soldigitalsolutions/xiaozhi-radio-server) to stream real-time Ogg/Opus transcoded radio stations directly on the robot.
* **MCP & Real-Time Voice AI:** Retains full Xiaozhi AI capabilities (wake word detection, WebSocket/MQTT+UDP audio streaming, and MCP device control).

---

### 🛠️ Hardware Architecture

* **Microcontroller:** ESP32-S3 (N16R8 / 1.54" TFT Cube variant)
* **Actuators & Motion:** Multi-servo drive system (PCA9685 / PWM control) + custom 3D-printed chassis
* **Audio & Display:** 1.54" IPS TFT screen, I2S microphone & amplifier/speaker
* **Power Supply:** Rechargeable 18650 Li-ion battery system with voltage/power management

---

### 🚀 Build & Flash (ESP-IDF)

1. Open the project folder in **VS Code** with the **ESP-IDF** extension installed (ESP-IDF v5.4+ recommended).
2. Select target `esp32s3` and verify the board configuration in `menuconfig` (`Xingzhi Cube 1.54 TFT WiFi`).
3. Build, flash, and monitor via USB:
   ```bash
   idf.py set-target esp32s3
   idf.py build flash monitor
   ```
4. Connect to the robot's captive Wi-Fi portal on first boot to configure your local network credentials (stored safely in NVS flash).

---

### 🙏 Credits & Support

* **Upstream Firmware:** Built upon the open-source [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) project (MIT License).
* **Maintained by:** **EPamar** — [Sol Digital Solutions](https://github.com/soldigitalsolutions)

If this firmware mod or hardware build helps your project, consider supporting my open-source work:

[![Support me on Ko-fi](https://img.shields.io/badge/Support%20me%20on-Ko--fi-FF5E5B?style=for-the-badge&logo=ko-fi&logoColor=white)](https://ko-fi.com/soldigitalsolutions)
