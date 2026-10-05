# Situation Selector training pipeline

Everything that runs on the PC: turning data into training streams, training, int8 export,
firmware parity checks, and the recorder / label tool for real flight data.

## Setup

```
python -m venv .venv && .venv\Scripts\activate      (Windows)
pip install tensorflow-cpu numpy pillow opencv-python pyserial
```
Checked with TensorFlow 2.21 on Python 3.13; `tensorflow-cpu==2.19.*` needs Python 3.9-3.12.
`gcc` on PATH (MSYS2 or WSL on Windows) is needed for the parity tests and for the tracker
table of `evaluate.py`; without it those parts are skipped.

## First model from a downloaded dataset

Run everything from this folder. `data/` and `run_*/` are in `.gitignore`.

```
1. python prepare_data.py visdrone D:\VisDrone2019-DET-train --map maps/visdrone.json --out data/visdrone
2. python train.py --data data/visdrone
3. python evaluate.py --run run_visdrone_v22_s1
4. python export_c.py --run run_visdrone_v22_s1
```

1. **prepare_data.py** reads the photos and boxes and writes training streams. Formats:
   `visdrone`, `yolo` (images + `.txt` labels), `voc` (Pascal VOC `.xml`, e.g. HERIDAL),
   `coco` (`--ann instances.json`), `wisard` (camera + thermal pairs). `--list-classes` prints the class names it finds; the
   `--map` file says which of them is which situation (`maps/` has examples; YOLO uses the
   class number as key). How a photo becomes a "flight" is explained at the top of the file.
   - **WiSARD** (real camera + thermal pairs): `python prepare_data.py wisard <root> --map maps/wisard.json --out data/wisard`.
     Use the folder of the synchronized pairs as `<root>`. The tool pairs camera and thermal
     pictures by file name, lines each pair up from the labelled people, and uses the real
     thermal picture as the third channel. Run it with `--list-classes` first: it prints how many
     pairs it found and three examples, so you see whether the pairing is right. It expects
     gray thermal pictures with warm = bright (`--black-hot` if warm is dark; colour-palette
     thermal pictures do not work). Written from the dataset's description and tested on made-up
     pairs, not on the real download.
   - A photo without boxes (YOLO: no label file) is empty ground. Add such photos of the
     terrain you will fly over: empty ground is what prevents false alarms.
   - `--gap-ms`: the sweep time range of the flight board. Default 150 350; use the value the
     board prints (`sweep ... ms`) once it is measured.
   - `--max-images` (default 1500) limits disk use: about 2-8 MB per photo with thermal.
   - **Thermal:** the model (`v22`) has a third image channel for a thermal camera. No public
     dataset of this kind has thermal, so a made-up thermal frame is generated from the boxes
     (warm people, some warm vehicles, warm spots with nobody there, flights where the ground is
     as warm as the people, coarse and fine sensors). It teaches the model how to combine both
     pictures, nothing more: real thermal recordings are needed before it can be trusted.
     `--no-thermal` leaves the channel out (then train with `--variant v21`, or keep `v22` and
     let it learn from the camera alone).
   - No public dataset has the landing marker: it is drawn onto some flights (`--marker-frac`),
     as an H in a square. Change `paste_marker()` if your marker looks different.
2. **train.py** prints one line per epoch: `val recall` (share of target frames found, mean over
   the situations) and `normal-fp` (share of empty frames called a situation). Roughly half an hour to an
   hour on a laptop CPU with the defaults.
3. **evaluate.py** writes `model_int8.tflite` and `eval.json`, and prints
   - recall per situation and **by target size in pixels**: this shows how small a person the
     model still finds, so whether 320x240 from 25-30 m is enough,
   - with thermal data: the same recall **with the thermal frame taken away** (camera only),
     so you see what each sensor contributes,
   - int8 against float (the int8 loop is the one the firmware runs),
   - the **tracker table**: events caught and false alarms per hour for each threshold `thr` and
     confirmation count `n_on`, per sweep, through the firmware's own `tracker.c`.
     Goal: caught >= 0.90 with <= 1 false alarm per hour.
4. Put the `thr` / `n_on` you chose into `situations.json`, then **export_c.py** writes
   `model_data.c`, `model_meta.h` and `situations.c` into the firmware. Build and flash; the
   board starts in SHADOW mode. After changing only `situations.json`:
   `python export_c.py --situations-only`.

Several datasets: prepare each into its own folder and train on one at a time, or copy the
image and label folders of the same format under one root before step 1.

## Real flight data

1. Flash the collect build (`COLLECT_MODE 1`), run `python recorder.py --out flights/`, fly.
   Press 1-3 when a situation is in view (rough marks). Frames go to disk while recording;
   after a crash run `python recorder.py --recover flights/`.
2. `python label_tool.py flights/flight_....npz`: mark events, and click the target centre at
   the start and at the end of EVERY event (and where it changes direction). Training uses only
   the frames between the first and the last click: the click tells which tile holds the target.
   Open flights without events once too, so they get an (empty) label file and count as "normal".
3. `python real_data.py flights/` prints usable frames and events per class per split.
4. `python prepare_data.py flights flights/ --out data/flights`, then steps 2-4 above.
   Frames whose telemetry is older than 500 ms are not used, so record with the flight
   controller connected.

## Files

| File | What it does |
|---|---|
| `situations.json` | The one situation list: classes, responses, tracker thresholds, typical target size. |
| `prepare_data.py` | Data source (image dataset, recorded flights, synthetic) -> dataset folder. |
| `dataset.py` | The dataset folder layout (memory-mapped frames, arrays, meta). |
| `maps/` | Class maps for `prepare_data.py --map`. |
| `models.py` | The step model (`v22`: camera + thermal, `v21`: camera only). One frame in, one frame out; h is an input and an output. Training unrolls this same model, so there is no weight copy at export. |
| `train.py` | Stage 1 (single frame) + stage 2 (streams with carried memory). |
| `evaluate.py` | Float and int8 accuracy, calibration (T), recall by size, tracker table. Writes `model_int8.tflite`. |
| `export_c.py` | Writes `model_data.c/.h`, `model_meta.h`, `situations.c` into the firmware `main/` folder. |
| `telemetry.py` | The 16 telemetry features. Must stay identical to `proto.c`. |
| `tiles.py` | Tile geometry and the speed rule. Must stay identical to `scan.c`. |
| `recorder.py` | Laptop side of the collect build: live view, rough marks, one `.npz` per flight. |
| `label_tool.py` | Labels events and target centres on a recorded flight. |
| `labels.py` | Label file model and events-to-frames conversion. |
| `real_data.py` | Recorded flights + labels -> one stream per tile, split by flight. |
| `synth.py` | Synthetic streams (5 demo classes). Only for testing the pipeline: `python prepare_data.py synth --out data/synth`. The model now in the firmware comes from these. |
| `fake_fc.py` | Bench test: pretends to be the flight controller on a USB-TTL adapter. |
| `trk_harness.c` | Runs the firmware tracker on probability streams (used by `evaluate.py`). |
| `test_data_tools.py`, `test_prepare_data.py`, `test_telemetry_parity.py` | Tests, no camera, window or download needed. |
