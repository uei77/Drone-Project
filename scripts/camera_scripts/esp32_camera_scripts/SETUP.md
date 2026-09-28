# ESP32-CAM Setup and First Test Guide

This document explains, step by step, how to set up the ESP32-CAM side of the project on a real board and how to test it.

> **Scope of this test:** Only the ESP32-CAM side is tested. No model is connected to the server yet; the server returns a fixed response for every frame (`"hold"`, confidence `0.0`). Testing with a model is the subject of the next stage.

This test verifies that:

- The code compiles and flashes to the board
- The camera starts and captures images
- Wi-Fi connects and reconnects after a drop
- Images are sent to the server as JSON and the response is received
- The UART output to the flight controller works (heartbeat and action lines)
- The system switches to a safe state (failsafe) when the server goes down

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
- Python (installed; for the test server)
- A phone's personal hotspot

> **Important:** The ESP32 can only connect to **2.4 GHz** Wi-Fi. The phone's hotspot must use the 2.4 GHz band. On an iPhone, turn on **"Maximize Compatibility"**.

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
    ├── idf_component.yml   External libraries (camera, cJSON)
    ├── config.h            All settings
    ├── main.c              Entry point (app_main)
    ├── wifi.c / .h         Wi-Fi connection
    ├── camera.c / .h       Camera start-up and recovery
    ├── fc_uart.c / .h      UART output and heartbeat to the flight controller
    └── net.c / .h          Image sending and response handling
```

The server files are in the `server/` folder one level up.

---

## 3. Running the Test Server on the Computer

For this test, the server runs directly on the computer instead of being deployed to Hugging Face.

### 3.1. Install the packages

In PowerShell, go to the `server` folder and install the packages:

```powershell
cd "C:\Users\Utku Ege\Documents\drone_project\scripts\camera_scripts\server"
python -m pip install -r requirements.txt
```

### 3.2. Start the server

```powershell
$env:API_TOKEN = "test-token"
$env:SAVE_FRAMES = "1"
python -m uvicorn app:app --host 0.0.0.0 --port 8000
```

- The `API_TOKEN` value must be **exactly the same** as the one in `config.h` on the ESP32 side.
- Because `SAVE_FRAMES = "1"`, every incoming frame is saved as a JPEG in the `server/frames/` folder. This is where we will check that the camera is actually capturing images.

To check that the server is running, open `http://localhost:8000/docs` in a browser. If the FastAPI documentation page opens, the server is ready.

On the first run, Windows Firewall may ask for permission for Python. Allow it on both **Private** and **Public** networks; a phone hotspot usually shows up as a "public network".

### 3.3. Choose how the ESP32 reaches the server

There are two options. Try A first; if it does not work, switch to B.

**Option A: Over the same hotspot (simplest)**

1. Connect the computer to the same phone hotspot as the ESP32.
2. In PowerShell, type `ipconfig` and note the **IPv4 address** of the Wi-Fi adapter (for example `192.168.43.25`).
3. Server address: `http://192.168.43.25:8000/frame` (with your own IP address).

This route uses HTTP, not HTTPS. That is fine for testing; the code supports both.

**Option B: Through a Cloudflare Tunnel (closer to real use)**

Use this route if the hotspot does not let devices see each other, or if you want to test the real HTTPS path. No account is needed.

```powershell
winget install --id Cloudflare.cloudflared
cloudflared tunnel --url http://localhost:8000
```

The command gives an address like `https://....trycloudflare.com`. The server address is that address with `/frame` added to the end. The address changes every time the command is run.

---

## 4. Configuring the ESP32

Fill in the values at the top of `main/config.h`:

```c
#define WIFI_SSID   "hotspot-name"
#define WIFI_PASS   "hotspot-password"
#define SERVER_URL  "http://192.168.43.25:8000/frame"   // the address from 3.3
#define API_TOKEN   "test-token"                        // same as in 3.2
#define DEVICE_ID   "drone1"
```

You do not need to change the other values for this test.

---

## 5. Building

1. In VS Code, open the `esp32_camera_scripts` folder with **File → Open Folder**.
2. Ctrl+Shift+P → **ESP-IDF: Select Current ESP-IDF Version** → choose the installed version.
3. Ctrl+Shift+P → **ESP-IDF: Set Espressif device target** → **esp32**.
4. Ctrl+Shift+P → **ESP-IDF: Build your project**.

The first build takes a few minutes. The camera and cJSON libraries are downloaded into the `managed_components/` folder during this step. If the build succeeds, the terminal shows a summary of how much space the program takes up.

> If you change `sdkconfig.defaults`, you need to delete the generated `sdkconfig` file and rebuild for the change to take effect.

---

## 6. Flashing the Board

### 6.1. Connection

**With the ESP32-CAM-MB:** Plug the ESP32-CAM into the MB board (the camera and the USB port on the MB board should face the same direction) and connect the USB cable to the computer. Most MB boards enter flashing mode automatically.

**With a USB-TTL (FTDI) adapter:**

| USB-TTL | ESP32-CAM |
|---|---|
| 5V | 5V |
| GND | GND |
| TX | U0R (GPIO3) |
| RX | U0T (GPIO1) |

Before flashing, **connect IO0 to GND** and press the RST button. When flashing is done, remove the IO0–GND connection and press RST again.

### 6.2. Select the port and flash

1. In Device Manager, look under **Ports (COM & LPT)** to see which COM port the board is using. If it does not appear, install the CH340 (or CP2102) driver.
2. Ctrl+Shift+P → **ESP-IDF: Select Port to Use** → choose that port.
3. Ctrl+Shift+P → **ESP-IDF: Build, Flash and Monitor**.

When flashing is done, the serial monitor opens automatically. Press **Ctrl+]** to exit the monitor.

If flashing stops with a "Failed to connect to ESP32" error: hold down the **IO0** button on the MB board, press **RST** once, release IO0, and start flashing again.

---

## 7. Test Steps

Each step describes what you should see. If a step does not work as expected, check section 8 before moving on.

### Test 1: Start-up and Wi-Fi

In the serial monitor you should see, in order:

```
I (...) wifi: Waiting for connection
I (...) wifi: Connected
```

If the camera started without problems, no camera error lines appear.

### Test 2: Sending images to the server

- In the PowerShell window where the server is running, `POST /frame HTTP/1.1" 200 OK` lines should scroll by several times per second.
- JPEG files should pile up in the `server/frames/` folder. Open a few of them and check that the image looks right. If the image is upside down or mirrored, that can be fixed later in the camera settings; it does not matter for this test.
- When everything is working, the ESP32's serial monitor **stays quiet**. The network module only prints warnings when something goes wrong.

### Test 3: Testing the action path with a fake response

Since there is no model, the server currently always returns `"hold"` with confidence `0.0`. Because that is below the confidence threshold (60%), the ESP32 does **not** forward this response to the flight controller, but it does count the link as healthy.

To see an action go all the way to the flight controller, temporarily change the `run_model` function in `server/app.py` like this:

```python
def run_model(img: Image.Image):
    return "left", 0.9
```

Stop the server (Ctrl+C) and start it again. Now every frame should produce an `ACT` line on the UART output (see 7.6). Undo this change after the test.

### Test 4: Failsafe when the server goes down

1. Stop the server with Ctrl+C.
2. `Request failed: ...` warnings should appear in the ESP32's serial monitor. The gap between requests grows gradually (up to 1 second).
3. Within about 2 seconds, the heartbeat lines should start saying `LOST` instead of `OK`.
4. Start the server again. The ESP32 should recover on its own; the heartbeat should go back to `OK`.

### Test 5: Wi-Fi drop

1. Turn off the phone's hotspot. The heartbeat should say `LOST`, and the program should not crash.
2. Turn the hotspot back on. The ESP32 should reconnect on its own and resume sending.

### 7.6. Watching the UART output

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
$HB,57,OK*0C
$ACT,12,left,90*6B
```

The two characters after the `*` sign are the line's checksum (XOR). Your counter and sequence numbers will be different, so your checksums will be different too.

**Without a second adapter (temporary workaround):** In `main/fc_uart.c`, add `#include "esp_log.h"` at the top, and in the `fc_send` function write this directly below the `uart_write_bytes` line:

```c
ESP_LOGI("fc", "%s", line);
```

The same lines will then also appear in the serial monitor. Since the heartbeat is written 5 times per second, the screen will scroll quickly. Remove this line after the test.

---

## 8. Common Problems

| Symptom | Likely cause | Fix |
|---|---|---|
| `Brownout detector was triggered` and constant rebooting | Insufficient power | Use a shorter, better-quality USB cable and try another USB port. On the drone, use a separate 5V BEC and a 470–1000 µF capacitor (`warning.txt`) |
| `Camera probe failed` or `camera init failed` | Camera cable not fully seated, or PSRAM disabled | Remove the camera's ribbon cable and reinsert it properly with its latch. Check that `CONFIG_SPIRAM=y` is in `sdkconfig.defaults` |
| Stuck at `Waiting for connection` | Wrong SSID/password, or the hotspot is 5 GHz | Check the values in `config.h` and set the hotspot to 2.4 GHz |
| `Request failed: ESP_ERR_HTTP_CONNECT` | The server cannot be reached | Check the IP address, the port (8000) and the firewall permission. If it still fails, switch to Option B (Cloudflare Tunnel) |
| `Status 401` | Token mismatch | `API_TOKEN` in `config.h` must match `$env:API_TOKEN` on the server |
| `Status 422` | The JSON format does not match what the server expects | Check that the ESP32 and server code are both up to date |
| `Bad response: ... ms` | The response arrives too late | The connection is slow. Set `CAM_FRAME_SIZE` to `FRAMESIZE_QVGA` in `config.h` |
| `seq does not fit` | A response to an old request arrived | Seeing it occasionally is normal; if it appears constantly, check that the server returns the `seq` value |
| `cJSON.h` not found during the build | ESP-IDF v6 is in use but cJSON has not been added | Check that the `espressif/cjson: "*"` line in `main/idf_component.yml` is enabled |
| A "multiple definition" error about cJSON during the build | ESP-IDF v5 is in use and cJSON has been added twice | cJSON is already included in v5; put a `#` at the start of the `espressif/cjson` line in `idf_component.yml` |
| `Failed to connect to ESP32` during flashing | The board is not in flashing mode | Follow the IO0/RST steps at the end of 6.2 |
| The COM port does not appear | USB driver missing, or the cable is charge-only | Install the CH340/CP2102 driver and use a data cable |

---

## 9. Next Steps

After these tests succeed, the next stages are:

1. **Collecting training data:** The frames collected during the `SAVE_FRAMES=1` tests are the first dataset for training the model. Training the model on the ESP32-CAM's own images gives better results in the field.
2. **Model integration:** The trained model will be connected to the `run_model` function in `server/app.py`.
3. **Moving the server online:** The server will be deployed to Hugging Face Spaces, and `SERVER_URL` will be replaced with the permanent HTTPS address.
4. **Flight controller side:** Code will be written on the flight controller that reads the UART lines, verifies their checksums and switches to a safe mode on `LOST`.
