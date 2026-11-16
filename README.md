# Upcycled AI Companion Robot — ESP32-S3 🤖♻️

An interactive, voice-controlled desktop and mobile companion robot built from **recycled materials** and powered by an **ESP32-S3** microcontroller. Engineered as a collaborative hardware and AI build by **EPamar** ([Sol Digital Solutions](https://github.com/soldigitalsolutions)) and **Fredy**.

---

### 🌟 What Makes This Robot Unique

Unlike standard dev-board builds, this project combines **sustainable upcycled mechanical fabrication** with **embodied Conversational AI**:

* **Upcycled Chassis & Mechanics:** Custom body, head, and articulated arms crafted from recycled materials.
* **Custom Animated Face UI:** Replaced the stock interface on the 1.54" TFT display with a custom expressive robot face synchronized with speech and interaction states.
* **Voice-Triggered Physical Actions (AI + MCP):** Speak naturally to the robot and it executes real-world movements—waving, turning its head, gesturing with its arms, and driving forward/backward/turning via custom MCP tool hooks in the firmware.
* **Full Kinematics & Locomotion:**
  * **Head & Arms:** Servo motors mapped to custom ESP32-S3 GPIO pins for expressive gestures and greetings.
  * **Drive System:** Traction motors controlled directly by the ESP32-S3 for physical movement.
* **Optional Live Radio Streaming:** Supports streaming Ogg/Opus internet radio via [xiaozhi-radio-server](https://github.com/soldigitalsolutions/xiaozhi-radio-server).

---

### 🛠️ Hardware & Control Architecture

* **Brain / MCU:** ESP32-S3 (`xingzhi-cube-1.54tft-wifi` base architecture)
* **Face Display:** 1.54" IPS TFT screen running custom facial animations
* **Actuators:**
  * Servo motors (head pan/tilt & articulated arms)
  * DC drive motors + motor driver stage for locomotion
* **Audio I/O:** Onboard I2S microphone (wake-word & voice recognition) + speaker output
* **Power:** Rechargeable 18650 Li-ion battery pack with onboard power management

---

### 🚀 Build & Flash (ESP-IDF)

1. Open the repository in **VS Code** with the **ESP-IDF** extension (v5.4+ recommended).
2. Set the target chip and verify pin assignments in `main/boards/nologo/` and `robot_controller`:
   ```bash
   idf.py set-target esp32s3
   idf.py build flash monitor
   ```
3. On first boot, connect to the robot's Wi-Fi captive portal to save your network credentials directly into NVS flash.

---

### 🤝 Credits & Support

* **Hardware Fabrication & Mechanical Assembly:** **Fredy** & **EPamar**
* **Firmware Customization, Custom Face & AI Integration:** **EPamar** — [Sol Digital Solutions](https://github.com/soldigitalsolutions)
* **Base Firmware:** Adapted from the open-source [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) project (MIT License).

If our upcycled robotics build or firmware modifications inspire your own projects, consider supporting our work:

[![Support me on Ko-fi](https://img.shields.io/badge/Support%20me%20on-Ko--fi-FF5E5B?style=for-the-badge&logo=ko-fi&logoColor=white)](https://ko-fi.com/soldigitalsolutions)
