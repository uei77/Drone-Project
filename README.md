# Drone Project

**An autonomous search-and-rescue drone that finds people on the ground with on-board vision.**

The drone flies a search pattern at 25-30 m, looks straight down, and recognises what matters
in a rescue: a person, a vehicle or wreckage, a landing marker. At its core is a small
*situation selector*: a neural network that decides, sweep by sweep, which situation the
drone is in, and a table that says how to respond to each one. The design is inspired by
System 1 models such as LAYA and Jev: fast, lightweight networks that react to what they see
without deliberate reasoning. Everything runs on a
low-cost microcontroller on the drone itself. There is no ground station in the loop and no video
link to lose: the drone finds, stops, and reports, and the operator decides what happens next.

> **Status:** in active development. The vision firmware, the flight-controller protocol and
> the complete training pipeline are implemented. Hardware integration and flight testing are
> the next milestones (see [Roadmap](#roadmap)).

## How it works

```
 camera 320x240 ─┐
 thermal frame ──┼─► tile scan ─► situation selector ─► tracker ─► policy ─► flight controller
 telemetry ──────┘   12 x 96x96    one pass per tile     confirm    respond     over UART
```

1. **Tile scan.** From 28 m a person is only a few pixels wide. Instead of shrinking the whole
   frame, the model looks at twelve overlapping 96x96 tiles of every frame, one after the
   other. One pass over all tiles is a *sweep*.
2. **Situation selector.** A compact recurrent network (about 37,000 parameters, under 80 KB as an
   int8 model) sees the current tile, the same tile one sweep earlier, an optional thermal
   channel, and the drone's telemetry. Each tile keeps its own memory, so the model judges
   motion over time rather than single frames.
3. **Tracker.** A detection has to hold for several sweeps before it counts as an event.
   Hysteresis and cooldowns keep single noisy frames from triggering a response.
4. **Policy.** Each situation has a response defined in one table, not in the network:

   | Situation | Response |
   |---|---|
   | `person` | Stop, hold position, report the location |
   | `vehicle_wreck` | Stop and report |
   | `landing_marker` | Centre over the marker, descend while correcting, land |

   Changing a response or a threshold never requires retraining.
5. **Flight controller link.** The vision board only *proposes*. Every event is acknowledged
   by the flight controller before any movement command is sent, and a silent or restarted
   vision board can never stop the mission.

### Designed to fail safe

- Three run modes: `OFF`, `SHADOW` (events are only logged) and `ACTIVE`. The board always
  starts in `SHADOW`.
- No decisions without fresh telemetry, and none outside an autonomous mission.
- A heartbeat tells the flight controller whether vision is running; losing it never pauses
  the flight.
- The firmware computes the highest ground speed at which detection still works for the
  measured sweep time, and warns when the drone exceeds it.

## Hardware

| Part | Role |
|---|---|
| ESP32-S3 camera board (8 MB PSRAM) | Flight target for the vision firmware |
| AI-Thinker ESP32-CAM | Development board (same firmware, slower sweep) |
| OV2640 camera, mounted looking down | 320x240 grayscale frames |
| Thermal sensor (to be selected) | Third image channel of the model |
| Flight controller running INAV | Flies the mission, applies the responses |

## Repository layout

```
scripts/camera_scripts/esp32_camera_scripts/   Vision firmware (ESP-IDF)
    main/        camera, tile scan, inference, tracker, policy, protocol, data collection
    test/        host unit tests for the hardware-independent modules
    SETUP.md     build, flash and bench-test guide (KURULUM.md: Turkish)
training/                                      Everything that runs on the PC
    prepare_data.py   image datasets or recorded flights -> training streams
    train.py          two-stage training with memory carried across time
    evaluate.py       float and int8 accuracy, calibration, threshold selection
    export_c.py       trained model + situation table -> firmware sources
    recorder.py       records camera frames and telemetry from the drone
    label_tool.py     labels events in recorded flights
    README.md         the training workflow, step by step
fc/                                            Flight-controller side (ArduPilot reference script)
```

## What is implemented

**Firmware**
- Camera capture, tile scan and TensorFlow Lite Micro inference for ESP32 and ESP32-S3
- Per-tile recurrent memory, telemetry features, optional thermal input channel
- Event tracker with hysteresis, priorities and cooldowns
- Response policy: report, hold, centre, guided descent and landing
- Checksummed line protocol with acknowledged events, heartbeat and run modes
- Data-collection build that streams frames and telemetry over Wi-Fi

**Training pipeline**
- Importers for common detection dataset formats (YOLO, COCO, Pascal VOC, VisDrone) and for
  paired camera + thermal datasets, including automatic alignment of the two images
- Conversion of still images into flight-like sequences with matching telemetry, scaled to the
  pixel size a target has from flight altitude
- Recording and labelling tools for real flight data, robust to packet loss and board restarts
- Stateful sequence training, int8 quantisation evaluated with the same loop the firmware runs,
  and threshold selection through the firmware's own tracker code
- One situation list (`training/situations.json`) shared by labelling, training and firmware

## Getting started

- **Firmware:** [`scripts/camera_scripts/esp32_camera_scripts/SETUP.md`](scripts/camera_scripts/esp32_camera_scripts/SETUP.md)
- **Training:** [`training/README.md`](training/README.md)

Quick check of the hardware-independent firmware modules on a PC:

```
cd scripts/camera_scripts/esp32_camera_scripts/test
gcc -std=c11 -I../main test_host.c ../main/proto.c ../main/tracker.c ../main/policy.c ../main/scan.c -lm -o test_host
./test_host
```

## Roadmap

1. **Board bring-up** on the ESP32-S3: sweep time, memory use and camera pixel scale.
2. **Thermal sensor:** select the sensor, write its driver, align it with the camera, and
   record paired data.
3. **INAV integration:** an adapter that reads telemetry from INAV and applies responses
   through it, with the safety limits enforced on the flight-controller side.
4. **Own flight data:** recordings from the drone's own camera at mission altitude, over the
   terrain it will search, to train the flight model.
5. **Shadow flights:** fly missions with responses disabled, review every event, feed false
   alarms back into training until they are rare enough.
6. **Active missions:** enable responses, starting with hold-and-report.
7. **Higher-resolution scan** (640x480) if the smallest targets need it.

## Safety

This is experimental software for an unmanned aircraft. It has not been validated for flight.
Fly only where it is legal and safe, keep manual control available at all times, and treat
every detection as a suggestion to a human operator, never as a decision.
