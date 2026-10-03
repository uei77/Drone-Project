"""Tests for recorder.py, label_tool.py, labels.py, real_data.py and tiles.py without a camera
or a window.
usage: python test_data_tools.py [path/to/esp32_camera_scripts/main]
(writes a temporary fake 320x240 flight, then deletes it; the firmware path is only needed
for the tile-geometry check against scan.c and needs gcc)
"""
import ctypes, glob, json, os, shutil, subprocess, sys, tempfile
import numpy as np
import recorder, label_tool, real_data, tiles
from labels import LabelDoc, load_situations, labels_path, marks_path

fails = 0
def check(c, what):
    global fails
    if not c:
        fails += 1; print("FAIL", what)

HERE = os.path.dirname(os.path.abspath(__file__))
tmp = tempfile.mkdtemp()
rng = np.random.default_rng(0)
N, FPS_MS, H, W = 200, 100, tiles.CAM_H, tiles.CAM_W          # 20 s recorded at 10 fps
frames = rng.integers(0, 255, (N, H, W), dtype=np.uint8)
T = lambda i: 1000 + i * FPS_MS

# ---------- tiles.py == firmware scan.c ----------
fw = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("FW_MAIN") or next(
    (p for p in (os.path.join(HERE, "..", "scripts", "camera_scripts", "esp32_camera_scripts", "main"),
                 os.path.join(HERE, "..", "fw", "main")) if os.path.exists(os.path.join(p, "scan.c"))), None)
if fw and shutil.which("gcc"):
    lib = os.path.join(tmp, "scan.so")
    subprocess.run(["gcc", "-O2", "-shared", "-fPIC", "-I", fw, os.path.join(fw, "scan.c"), "-o", lib], check=True)
    so = ctypes.CDLL(lib)
    fl2 = ctypes.c_float * 2
    for t in range(tiles.SCAN_TILES):
        x0, y0 = ctypes.c_int(), ctypes.c_int()
        so.scan_tile_origin(t, ctypes.byref(x0), ctypes.byref(y0))
        check((x0.value, y0.value) == tiles.ORIGINS[t], f"tile {t} origin matches scan.c")
        tp, fp = fl2(0.3, -0.6), fl2()
        so.scan_to_frame(t, tp, fp)
        check(np.allclose(list(fp), tiles.tile_to_frame(t, 0.3, -0.6), atol=1e-6), f"tile {t} mapping matches scan.c")
    so.scan_speed_limit.restype = ctypes.c_float
    so.scan_speed_limit.argtypes = [ctypes.c_float, ctypes.c_float]
    check(abs(so.scan_speed_limit(28.0, 0.2) - tiles.speed_limit(28.0, 0.2)) < 1e-4, "speed limit matches scan.c")
    thf = rng.integers(0, 255, (H // 2, W // 2), dtype=np.uint8)
    buf = (ctypes.c_uint8 * (96 * 96))()
    for t in (0, 5, 11):
        so.scan_thermal_tile(thf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), t, buf)
        check(np.array_equal(np.frombuffer(buf, np.uint8).reshape(96, 96), tiles.thermal_crop(thf, t)),
              f"thermal tile {t} matches scan.c")
    print("tile geometry checked against", os.path.normpath(fw))
else:
    print("NOTE: scan.c or gcc not found, tile geometry not compared with the firmware")
check(len(tiles.ORIGINS) == 12 and tiles.ORIGINS[-1] == (W - 96, H - 96), "12 tiles reach the far corner")
bx, by = tiles.frame_to_tile(5, *tiles.tile_to_frame(5, 0.25, -0.5))
check(abs(bx - 0.25) < 1e-9 and abs(by + 0.5) < 1e-9, "frame <-> tile round trip")

# ---------- recorder: packets exactly as collect.c sends them ----------
rec_dir = os.path.join(tmp, "rec"); os.makedirs(rec_dir)
rx = recorder.Receiver(0, rec_dir)
NCH = W * H // recorder.SLICE
def packets(fid, esp_ms, age, drop=(), img=None):
    tel = (esp_ms, 28.0, 90.0, 0.1, 0.0, 3.0, 5.0, 2, 10, 95.0, 1)
    raw = (frames[fid] if img is None else img).tobytes()
    for c in range(NCH):
        if c not in drop:
            yield recorder.HDR.pack(recorder.MAGIC, fid, c, NCH, recorder.SLICE, W, H, esp_ms, age, *tel) \
                  + raw[c * recorder.SLICE:(c + 1) * recorder.SLICE]
LOST = 120
for i in range(N):
    age = 0xFFFFFFFF if i < 10 else (900 if 100 <= i < 105 else 40)   # no tel at start, stale later
    drop = (3,) if i == 50 else tuple(range(20)) if i == LOST else ()
    for p in packets(i, T(i), age, drop):
        rx.feed(p)
    if i == 60:                                   # a late copy of an old packet is not a restart
        check(rx.feed(next(packets(58, T(58), 40))) is False and rx.writer is not None, "late packet ignored")
check(recorder.HDR.size == 64 and NCH == 64, "packet layout as in collect.c")
check(rx.count == N - 1 and rx.lost == 1 and rx.partial == 1,
      "a frame with 20 lost packets is dropped, one with 1 lost packet is kept")
check(rx.feed(b"junk") is False, "junk packet ignored")
check(os.path.getsize(rx.writer.base + ".frames.tmp") >= (N - 12) * W * H, "frames are on disk while recording")
img = recorder.render(rx.last, rx.count, 9.8, rx.lost, 20.0, [dict(t_ms=1, key="1")])
check(img.shape == (480, 900, 3), "live view image")
check(recorder.render(None, 0, 0.0, 0, 0.0, []).shape == (480, 900, 3), "live view before the first frame")
rx.writer.marks = [dict(t_ms=T(30), key="1"), dict(t_ms=T(75), key="note")]
rx.rotate()
check(len(rx.files) == 1 and not glob.glob(os.path.join(rec_dir, "*.tmp")), "one flight file, no temp files left")
os.replace(marks_path(rx.files[0]), os.path.join(tmp, "flight_a_marks.json"))
path = os.path.join(tmp, "flight_a.npz"); os.replace(rx.files[0], path)
d = np.load(path)
check(d["frames"].shape == (N - 1, H, W) and d["tel"].shape == (N - 1, 11), "npz layout")
check(np.array_equal(d["frames"][5], frames[5]) and np.all(np.diff(d["esp_ms"].astype(np.int64)) > 0),
      "frames byte-exact and in time order")
miss = d["missing"]
check(miss.shape == (N - 1, NCH) and miss.sum() == 1 and miss[50, 3], "lost packet recorded")
r0 = 3 * recorder.SLICE // W
check(not d["frames"][50, r0 + 1].any() and np.array_equal(d["frames"][50, r0 + 5], frames[50, r0 + 5]),
      "lost part is black, the rest of the frame is intact")
check(real_data.tile_has_hole(miss[50:51], 0)[0] and not real_data.tile_has_hole(miss[50:51], 4)[0],
      "a lost packet marks only the tiles it covers")

# the ESP32 restarts in the air: frame numbers and clock start again -> second flight file
rx.sock.close()
rx = recorder.Receiver(0, rec_dir)
for i in range(30):
    for p in packets(i, T(i), 40): rx.feed(p)
for i in range(12):
    for p in packets(i, 300 + i * FPS_MS, 40): rx.feed(p)
rx.rotate()
n_fr = [len(np.load(f)["esp_ms"]) for f in rx.files]
check(n_fr == [30, 12], "restart of the ESP32 starts a new flight file")
check(all(np.all(np.diff(np.load(f)["esp_ms"].astype(np.int64)) > 0) for f in rx.files), "time never runs backwards in a file")
rx.sock.close()

# crash while recording: the temp files are turned into a flight by --recover
crash = os.path.join(tmp, "crash"); os.makedirs(crash)
wr = recorder.Writer(crash, (H, W), NCH)
for i in range(3):
    wr.add(dict(frame_id=i, esp_ms=T(i), age=40, tel=(T(i), 28.0, 90.0, 0, 0, 3, 5, 2, 10, 95, 1),
                missing=np.zeros(NCH, bool), frame=frames[i]))
wr.frames.flush(); wr.meta.flush()
with open(wr.base + ".frames.tmp", "r+b") as f:
    f.truncate(3 * W * H - 100)                   # the last frame was cut off
wr.frames.close(); wr.meta.close()
check(recorder.recover(crash) == 1, "recover finds the unfinished recording")
rec = glob.glob(os.path.join(crash, "flight_*.npz"))
check(len(rec) == 1 and np.array_equal(np.load(rec[0])["frames"], frames[:2]), "recovered the complete frames")

# ---------- label tool: scripted session ----------
s = label_tool.Session(path, labeler="test")
check(len(s.marks) == 2 and (s.sw, s.scale) == (640, 2), "marks loaded, 2x view")
s.seek_ms(T(30)); check(s.key_class("1"), "start person")
s.seek_ms(T(60)); check(s.end_event(), "end person")
# target walks from frame px (40, 40) [tile 0 only] to px (300, 220) [tile 11 only]
s.seek_ms(T(35)); check(s.click_image(40 * 2, 40 * 2), "target at 35")
s.seek_ms(T(55)); s.click_image(300 * 2, 220 * 2)
s.seek_ms(T(150)); s.key_class("3"); s.seek_ms(T(170)); s.end_event()        # vehicle_wreck, no target point
s.seek_ms(T(125)); s.key_class("2"); s.seek_ms(T(155)); s.end_event()        # overlaps -> trims arac
ev = {e["situation"]: e for e in s.doc.events}
check(set(ev) == {"person", "landing_marker", "vehicle_wreck"}, "three events")
check(ev["vehicle_wreck"]["start_ms"] == T(155) + 1, "overlap trimmed")
check(abs(ev["person"]["targets"][0]["x"] - (40 / 160 - 1)) < 2e-3 and
      abs(ev["person"]["targets"][0]["y"] - (40 / 120 - 1)) < 2e-3, "click -> frame coordinates")
s.seek_ms(T(62)); check(s.boundary("end") and ev["person"]["end_ms"] == T(62), "move end")
s.seek_ms(T(0)); s.toggle_exclude(); s.seek_ms(T(5)); s.toggle_exclude()
check(len(s.doc.exclude) == 1, "exclude range")
s.seek_ms(T(100)); check(not s.click_image(10, 10), "no target outside an event")
s.seek_ms(T(25)); s.jump(1); check(s.t == T(30), "jump to next event")
check(label_tool.render(s).shape == (480 + label_tool.TL_H, 940, 3), "label window image")
s2 = label_tool.Session(path)
check(json.dumps(s2.doc.events) == json.dumps(s.doc.events), "labels survive save and reload")
check(not os.path.exists(s.path + ".tmp"), "atomic save leaves no temp file")

# ---------- real_data: events in time -> per-tile training streams ----------
version, classes = load_situations()
st = real_data.load_flight(path, classes, version, np.random.default_rng(1), gap_ms=(150, 350))
check(len(st) == 12 and all(x["frames"].shape[1:] == (96, 96) for x in st), "twelve tile streams")
t = st[0]["esp_ms"]
gaps = np.diff(t)
check(gaps.min() >= 150 and gaps.max() <= 450, "frames picked at the sweep gap")
check(st[0]["tel"].shape == (len(t), 16) and np.abs(st[0]["tel"]).max() <= 1.0, "telemetry features bounded")
check(abs(st[0]["tel"][5, 8] - min(gaps[4] / 300.0, 1.0)) < 1e-5, "dt feature = real gap (clamped)")
i35 = int(np.argmin(np.abs(t - T(36))))           # shortly after the first click: target near px (40,40)
check(st[0]["y"][i35] == 1 and st[0]["has"][i35] == 1, "tile 0 sees the person")
check(all(st[k]["y"][i35] == 0 for k in range(1, 12)), "other tiles are normal at that moment")
px = (np.interp(t[i35], [T(35), T(55)], [40, 300]), np.interp(t[i35], [T(35), T(55)], [40, 220]))
check(np.allclose(st[0]["pos"][i35], [px[0] / 48 - 1, px[1] / 48 - 1], atol=0.02), "target in tile coordinates")
i55 = int(np.argmin(np.abs(t - T(55))))
check(st[11]["y"][i55] == 1 and st[0]["y"][i55] == 0, "target moved to the last tile")
late = (t > T(55) + 400) & (t <= T(62))            # in the event, but after the last click
check(late.any() and all((x["w"][late] == 0).all() and (x["y"][late] == 0).all() for x in st),
      "event frames away from any click are not used (target position unknown)")
labelled = sum(int(x["y"][i].item() == 1) for x in st for i in range(len(t)) if T(31) <= t[i] <= T(62))
check(labelled > 0 and all((x["w"][(t >= T(63)) & (t <= T(64))] == 0).all() for x in st), "tail after the event has weight 0")
arac = (t >= T(160)) & (t <= T(170))
check(all((x["w"][arac] == 0).all() and (x["y"][arac] == 0).all() for x in st), "event without target point: weight 0")
check(all((x["w"][t <= T(5)] == 0).all() for x in st), "excluded range has weight 0")
check(all((x["w"][(t >= T(100)) & (t < T(105))] == 0).all() for x in st), "stale telemetry has weight 0")
x0, y0 = tiles.ORIGINS[7]
check(np.array_equal(st[7]["frames"][3], d["frames"][np.searchsorted(d["esp_ms"], t[3])][y0:y0 + 96, x0:x0 + 96]),
      "tile pixels are an exact crop")
# a target just outside a tile makes that tile ambiguous, not "normal"
edge_seen = any(((x["w"] == 0) & (x["y"] == 0) & (t > T(36)) & (t < T(54))).any() for x in st)
check(edge_seen, "tiles the target only touches get weight 0")

# every frame picked: the frame with the lost packet and the one after it
se = real_data.load_flight(path, classes, version, np.random.default_rng(1), gap_ms=(50, 60))
j = int(np.searchsorted(se[0]["esp_ms"], T(50)))
check((se[0]["w"][j:j + 2] == 0).all() and se[4]["w"][j] == 1 and se[0]["w"][j + 2] == 1,
      "lost packet: weight 0 for the tiles it touches, this frame and the next")
sm = real_data.summary(st, classes)
check(sm["person"]["events"] == 1 and sm["vehicle_wreck"]["events"] == 0 and sm["landing_marker"]["events"] == 0,
      "an event counts once however many tiles it crosses; events without usable frames do not count")

# ---------- split by flight ----------
for name in "bcdefg":
    shutil.copy(path, os.path.join(tmp, f"flight_{name}.npz"))
    LabelDoc(f"flight_{name}.npz", version).save(labels_path(os.path.join(tmp, f"flight_{name}.npz")))
tr, va, te, _ = real_data.load_flights(tmp)
fl = lambda part: {x["name"] for x in part}
check(len(tr) == 60 and len(va) == 12 and len(te) == 12, "5 / 1 / 1 flights, 12 streams each")
check(not (fl(tr) & fl(va)) and not (fl(tr) & fl(te)) and not (fl(va) & fl(te)), "no flight in two splits")
try:
    os.remove(labels_path(os.path.join(tmp, "flight_b.npz")))
    real_data.load_flights(tmp); check(False, "missing label file must be an error")
except FileNotFoundError:
    pass

# ---------- flights -> dataset folder (prepare_data.py flights) ----------
import dataset
LabelDoc("flight_b.npz", version).save(labels_path(os.path.join(tmp, "flight_b.npz")))
out = os.path.join(tmp, "ds")
r = subprocess.run([sys.executable, os.path.join(HERE, "prepare_data.py"), "flights", tmp, "--out", out, "--len", "16"],
                   capture_output=True, text=True)
check(r.returncode == 0, "prepare_data.py flights runs: " + r.stderr[-300:])
dtr = dataset.load(out, "train")
check(dataset.read_meta(out)["classes"][1] == "person" and dtr["frames"].shape[1:] == (16, 96, 96)
      and len(dtr["y"]) % 12 == 0 and len(dataset.load(out, "val")["y"]) > 0, "dataset folder from flights")

shutil.rmtree(tmp)
print("FAILED" if fails else "OK", f"({fails} failed)")
raise SystemExit(1 if fails else 0)
