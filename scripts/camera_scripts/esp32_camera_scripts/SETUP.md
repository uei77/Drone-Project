# ESP32-CAM Setup and First Test Guide

This document explains step by step how to set up and test the ESP32-CAM side of the project on a real board.

> **Scope of this version:** The server has been removed. The decision model will later run directly on the ESP32; there is no model in this version yet. At start-up the ESP32 starts the camera, captures one frame to confirm the camera works, and sends heartbeats to the flight controller. Because there is no model producing decisions, the heartbeat always says `LOST`; this is expected and safe.

This test verifies that:

- The code compiles and flashes to the board
- The camera starts and captures images
- The UART output to the flight controller works (heartbeat)

---

## 1. Requirements

### Hardware

| Part | Description |
|---|---|
| ESP32-CAM (AI-Thinker) | Model with the OV2640 camera |
| ESP32-CAM-MB programmer board | USB board that plugs under the ESP32-CAM. A USB-TTL (FTDI) adapter also works |
| USB cable | A short, good-quality cable that supports data transfer |
| A second USB-TTL adapter *(optional)* | To view the UART output going to the flight controller |

### Software

- ESP-IDF and the VS Code ESP-IDF extension (installed)

---

## 2. Project Structure

```
esp32_camera_scripts/
├── CMakeLists.txt          Project definition
├── sdkconfig.defaults      Hardware and system settings (PSRAM, watchdog, etc.)
├── KURULUM.md              Turkish version of this guide
├── SETUP.md                This document
└── main/
    ├── CMakeLists.txt      List of .c files to compile
    ├── idf_component.yml   External libraries (camera)
    ├── config.h            All settings
    ├── main.c              Entry point (app_main)
    ├── wifi.c / .h         Wi-Fi connection (not called yet; kept for data collection)
    ├── camera.c / .h       Camera start-up and recovery
    └── fc_uart.c / .h      UART output and heartbeat to the flight controller
```

---

## 3. Configuring the ESP32

There is nothing to fill in for this version. `WIFI_SSID` and `WIFI_PASS` in `main/config.h` are kept for transferring training data to the computer later; they are not used right now.

> **Warning:** Before writing your real Wi-Fi password into `config.h`, run this command so the password never reaches git: `git update-index --skip-worktree scripts/camera_scripts/esp32_camera_scripts/main/config.h`

---

## 4. Building

1. In VS Code, open the `esp32_camera_scripts` folder with **File → Open Folder**.
2. Ctrl+Shift+P → **ESP-IDF: Select Current ESP-IDF Version** → choose the installed version.
3. Ctrl+Shift+P → **ESP-IDF: Set Espressif device target** → **esp32**.
4. Ctrl+Shift+P → **ESP-IDF: Build your project**.

The first build takes a few minutes. The camera library is downloaded into the `managed_components/` folder during this step. If the build succeeds, the terminal shows a summary of how much space the program takes up.

> If you change `sdkconfig.defaults`, you need to delete the generated `sdkconfig` file and rebuild for the change to take effect.

---

## 5. Flashing the Board

### 5.1. Connection

**With the ESP32-CAM-MB:** Plug the ESP32-CAM into the MB board (the camera and the USB port on the MB board should face the same direction) and connect the USB cable to the computer. Most MB boards enter flashing mode automatically.

**With a USB-TTL (FTDI) adapter:**

| USB-TTL | ESP32-CAM |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | U0R (GPIO3) |
| RX | U0T (GPIO1) |

Before flashing, **connect IO0 to GND** and press the RST button. When flashing is done, remove the IO0–GND connection and press RST again.

### 5.2. Select the port and flash

1. In Device Manager, look under **Ports (COM & LPT)** to see which COM port the board is using. If it does not appear, install the CH340 (or CP2102) driver.
2. Ctrl+Shift+P → **ESP-IDF: Select Port to Use** → choose that port.
3. Ctrl+Shift+P → **ESP-IDF: Build, Flash and Monitor**.

When flashing is done, the serial monitor opens automatically. Press **Ctrl+]** to exit the monitor.

If flashing stops with a "Failed to connect to ESP32" error: hold down the **IO0** button on the MB board, press **RST** once, release IO0, and start flashing again.

---

## 6. Test Steps

Each step describes what you should see. If a step does not work as expected, check section 7 before moving on.

### Test 1: Start-up and camera

In the serial monitor you should see a line like this (your resolution and byte count may differ):

```
I (...) main: Camera OK: 640x480, 18234 bytes
```

This line shows that the camera started and actually captured a frame. If you see `Camera could not capture a frame` instead, the ESP32 tries to restart the camera; if that does not help, check section 7.

### Test 2: Heartbeat

The UART output (see 6.3) should show a `$HB,<counter>,LOST*<checksum>` line 5 times per second. Since there is no decision model yet, the status is always `LOST`; the flight controller should stay in a safe mode (hold) in this state. Once the model is added, the status will be `OK` whenever the model has run within the last 2 seconds.

### 6.3. Watching the UART output

The lines going to the flight controller come out of the **GPIO14** pin; they do not appear in the serial monitor. There are two ways to see them:

**With a second USB-TTL adapter (recommended):**

| USB-TTL | ESP32-CAM |
|---|---|
| RX | GPIO14 |
| GND | GND |

Open the adapter's COM port at 115200 baud in a serial terminal (PuTTY or the VS Code "Serial Monitor" extension). You should see lines like these:

```
$HB,0,LOST*3E
$HB,1,LOST*3F
$HB,2,LOST*3C
```

The two characters after the `*` sign are the line's checksum (XOR). Your counter will be different, so your checksums may be different too.

**Without a second adapter (temporary workaround):** In `main/fc_uart.c`, add `#include "esp_log.h"` at the top, and in the `fc_send` function write this directly below the `uart_write_bytes` line:

```c
ESP_LOGI("fc", "%s", line);
```

The same lines will then also appear in the serial monitor. Since the heartbeat is written 5 times per second, the screen will scroll quickly. Remove this line after the test.

---

## 7. Common Problems

| Symptom | Likely cause | Fix |
|---|---|---|
| `Brownout detector was triggered` and constant rebooting | Insufficient power | Use a shorter, better-quality USB cable and try another USB port. On the drone, use a separate 5V BEC and a 470–1000 µF capacitor (`warning.txt`) |
| `Camera probe failed`, `camera init failed` or `Camera could not capture a frame` | Camera cable not fully seated, or PSRAM disabled | Remove the camera's ribbon cable and reinsert it properly with its latch. Check that `CONFIG_SPIRAM=y` is in `sdkconfig.defaults` |
| `Failed to connect to ESP32` during flashing | The board is not in flashing mode | Follow the IO0/RST steps at the end of 5.2 |
| The COM port does not appear | USB driver missing, or the cable is charge-only | Install the CH340/CP2102 driver and use a data cable |

---

## 8. Next Steps

After these tests succeed, the next stages are:

1. **Adding the decision model to the ESP32:** The model will run directly on the ESP32 with TensorFlow Lite Micro. The camera will switch to 96×96 grayscale frames at this stage. First, the model will be added with random weights to measure its run time and memory use on the board.
2. **Collecting training data:** Frames from the drone's own camera will be transferred to the computer (a small recording script over Wi-Fi, or an SD card). `wifi.c` stays in the project for this.
3. **Training the model on the computer:** The model will be trained on the computer, converted to int8, embedded into the firmware and flashed to the board over USB.
4. **Flight controller side:** Code will be written on the flight controller that reads the UART lines, verifies their checksums and switches to a safe mode on `LOST`.
