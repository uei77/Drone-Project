"""Tests for prepare_data.py and dataset.py on a tiny made-up image dataset (no download needed).
usage: python test_prepare_data.py
"""
import json, os, shutil, subprocess, sys, tempfile
import numpy as np
import cv2
import dataset, prepare_data as pd, tiles

fails = 0
def check(c, what):
    global fails
    if not c:
        fails += 1; print("FAIL", what)

HERE = os.path.dirname(os.path.abspath(__file__))
tmp = tempfile.mkdtemp()
rng = np.random.default_rng(0)

def photo(w, h, boxes):
    """dark noisy ground, one bright rectangle per box"""
    img = rng.integers(20, 60, (h, w)).astype(np.uint8)
    for x, y, bw, bh in boxes:
        img[y:y + bh, x:x + bw] = 250
    return img

# ---------- the four readers give the same boxes ----------
W, H, BOX = 800, 600, (300, 200, 40, 60)
for fmt in ("yolo", "visdrone", "voc", "coco"):
    os.makedirs(os.path.join(tmp, fmt, "images")); os.makedirs(os.path.join(tmp, fmt, "labels" if fmt == "yolo" else "annotations"))
    cv2.imwrite(os.path.join(tmp, fmt, "images", "a.jpg"), photo(W, H, [BOX]))
x, y, bw, bh = BOX
open(os.path.join(tmp, "yolo", "labels", "a.txt"), "w").write(f"0 {(x + bw / 2) / W} {(y + bh / 2) / H} {bw / W} {bh / H}\n")
open(os.path.join(tmp, "visdrone", "annotations", "a.txt"), "w").write(f"{x},{y},{bw},{bh},1,1,0,0\n10,10,50,50,0,0,0,0,\n")
open(os.path.join(tmp, "voc", "annotations", "a.xml"), "w").write(
    f"<annotation><filename>a.jpg</filename><object><name>human</name><bndbox><xmin>{x}</xmin><ymin>{y}</ymin>"
    f"<xmax>{x + bw}</xmax><ymax>{y + bh}</ymax></bndbox></object></annotation>")
json.dump(dict(images=[dict(id=7, file_name="a.jpg")], categories=[dict(id=1, name="person")],
               annotations=[dict(image_id=7, category_id=1, bbox=[x, y, bw, bh])]),
          open(os.path.join(tmp, "coco", "annotations", "inst.json"), "w"))
it = pd.read_yolo(os.path.join(tmp, "yolo"))[0]
check(it["norm"] and it["boxes"][0][0] == "0" and abs(it["boxes"][0][1] * W - (x + bw / 2)) < 1e-6, "yolo reader")
it = pd.read_visdrone(os.path.join(tmp, "visdrone"))[0]
check(it["boxes"] == [("pedestrian", x, y, bw, bh), ("ignored regions", 10, 10, 50, 50)], "visdrone reader")
it = pd.read_voc(os.path.join(tmp, "voc"))[0]
check(it["boxes"] == [("human", x, y, bw, bh)], "voc reader")
it = pd.read_coco(os.path.join(tmp, "coco"), os.path.join(tmp, "coco", "annotations", "inst.json"))[0]
check(it["boxes"] == [("person", x, y, bw, bh)] and it["path"].endswith("a.jpg"), "coco reader")

# ---------- mirrored canvas: every box copy lies on an object ----------
small = photo(200, 150, [(30, 100, 20, 10)])
canvas, objs = pd.make_canvas(small, [(1, 30, 100, 20, 10)], 1.0)
check(canvas.shape == (480, 640) and len(objs) >= 9, "a small photo is mirrored up to twice the frame")
on = [canvas[int(round(b[2])) + 2:int(round(b[2] + b[4])) - 2, int(round(b[1])) + 2:int(round(b[1] + b[3])) - 2]
      for b in objs if b[1] >= 0 and b[2] >= 0 and b[1] + b[3] <= 640 and b[2] + b[4] <= 480]
check(len(on) >= 6 and all(o.size and o.min() == 250 for o in on), "mirrored boxes sit exactly on the mirrored objects")
check(int((canvas == 250).sum()) == sum(int(np.clip(min(b[1] + b[3], 640) - max(b[1], 0), 0, None) *
      np.clip(min(b[2] + b[4], 480) - max(b[2], 0), 0, None)) for b in objs), "no object without a box")
big, objs = pd.make_canvas(photo(1600, 1200, []), [(1, 800, 600, 80, 80)], 0.5)
check(big.shape == (600, 800) and np.allclose(objs[0], [1, 400, 300, 40, 40]), "a large photo is only scaled")

# ---------- tile labels ----------
prio = [0, 3, 1, 2]
o = np.array([(1, 35, 35, 10, 10)], np.float32)                       # centre (40, 40): only tile 0
y, pos, has, w, size = pd.tile_labels(o, 0, 0, prio)
check(y[0] == 1 and y[1:].sum() == 0 and has[0] == 1 and abs(size[0] - 10) < 1e-5, "target centre inside tile 0")
check(np.allclose(pos[0], [40 / 48 - 1, 40 / 48 - 1]) and (w == 1).all(), "position in tile coordinates")
y, pos, has, w, size = pd.tile_labels(o, 10, 20, prio)                 # the frame moved: target at (30, 20)
check(np.allclose(pos[0], [30 / 48 - 1, 20 / 48 - 1]), "frame offset is taken off")
o = np.array([(1, 80, 30, 10, 10)], np.float32)                       # centre x = 85: in tile 0 and in tile 1 (75..171)
y, _, _, w, _ = pd.tile_labels(o, 0, 0, prio)
check(y[0] == 1 and y[1] == 1, "overlap zone: both tiles see the target")
o = np.array([(1, 100, 30, 10, 10)], np.float32)                      # centre x = 105: 9 px right of tile 0
y, _, _, w, _ = pd.tile_labels(o, 0, 0, prio)
check(y[0] == 0 and w[0] == 0 and y[1] == 1 and w[1] == 1, "partly visible in a tile: weight 0 there")
o = np.array([(1, 40, 40, 2, 2), (-1, 230, 150, 60, 60)], np.float32)
y, _, _, w, _ = pd.tile_labels(o, 0, 0, prio)
check(y.sum() == 0 and w[0] == 0 and w[11] == 0 and w[4] == 1, "too small to see and ignore regions: weight 0")
o = np.array([(3, 30, 30, 30, 30), (1, 50, 50, 8, 8)], np.float32)
check(pd.tile_labels(o, 0, 0, prio)[0][0] == 1, "two targets in a tile: the higher priority wins")

# ---------- flight path and telemetry ----------
p = pd.fly(900, 700, 200, np.random.default_rng(3))
check(p.min() >= 0 and (p[:, 0] <= 900 - 320).all() and (p[:, 1] <= 700 - 240).all(), "frame stays on the canvas")
check(np.abs(np.diff(p, axis=0)).max() <= tiles.TILE / 2 + 6, "at most half a tile per step")
up = np.array([(100, 400 - 20 * i) for i in range(10)]); right = np.array([(100 + 20 * i, 200) for i in range(10)])
tel, dt = pd.flight_telemetry(up, 28.0, (200, 200), np.random.default_rng(1))
check((tel[1:, 9] > 0.2).all() and (np.abs(tel[1:, 10]) < 1e-5).all(), "window moves up = flying forward")
check(abs(tel[3, 9] - (20 * tiles.metres_per_px(28.0) / 0.2) / tel[3, 0] / 50 / 0.5) < 0.02, "ego-motion value = speed / altitude")
tel, dt = pd.flight_telemetry(right, 28.0, (200, 200), np.random.default_rng(1))
check((tel[1:, 10] > 0.2).all() and (np.abs(tel[1:, 9]) < 1e-5).all(), "window moves right = flying right")
check(np.allclose(dt, 200) and abs(tel[2, 8] - 200 / 300) < 1e-5 and np.abs(tel).max() <= 1, "dt feature, all bounded")

# ---------- landing marker ----------
c = np.full((480, 640), 100, np.float32)
box = pd.paste_marker(c, 300, 200, 26, np.random.default_rng(5))
check(box is not None and abs(box[0] + 13 - 300) < 1e-6 and (c != 100).sum() > 26 * 26 * 0.9, "marker drawn where the box says")
check(pd.paste_marker(c, 5, 5, 26, np.random.default_rng(5)) is None, "marker off the canvas is not drawn")

# ---------- made-up thermal ----------
cv_, ob_ = pd.make_canvas(photo(1600, 1200, []), [(1, 800, 600, 80, 80), (1, 200, 200, 80, 80)], 0.5)
hot = 0
for seed in range(20):
    f = pd.thermal_field(cv_, ob_, 1, 3, np.random.default_rng(seed))
    u = pd.thermal_u8(f)
    hot += u[160, 210] > 150 and u[60, 60] > 150            # the two people (thermal grid = canvas / 2)
check(f.shape == (300, 400) and u.dtype == np.uint8 and abs(float(np.median(u)) - 128) <= 1, "thermal frame: half resolution, median at 128")
check(12 <= hot <= 19, f"people are warm spots in most flights, not in all ({hot}/20)")
th = np.arange(120 * 160, dtype=np.uint16).reshape(120, 160)
c5 = tiles.thermal_crop(th, 5)
check(c5.shape == (96, 96) and c5[0, 0] == 36 * 160 + 37 and c5[0, 1] == c5[0, 2] == 36 * 160 + 38, "thermal tile: camera pixel -> thermal pixel under it")

# ---------- whole tool on a made-up YOLO dataset ----------
root = os.path.join(tmp, "ds")
os.makedirs(os.path.join(root, "images")); os.makedirs(os.path.join(root, "labels"))
for i in range(30):
    bs = [(int(rng.integers(50, 1100)), int(rng.integers(50, 800)), 20, 40) for _ in range(int(rng.integers(0, 4)))]
    cv2.imwrite(os.path.join(root, "images", f"{i:03d}.jpg"), photo(1280, 960, bs))
    with open(os.path.join(root, "labels", f"{i:03d}.txt"), "w") as f:
        for x, y, bw, bh in bs:
            f.write(f"0 {(x + bw / 2) / 1280} {(y + bh / 2) / 960} {bw / 1280} {bh / 960}\n")
out = os.path.join(tmp, "out")
r = subprocess.run([sys.executable, os.path.join(HERE, "prepare_data.py"), "yolo", root, "--map",
                    os.path.join(HERE, "maps", "person_yolo.json"), "--out", out, "--len", "16",
                    "--marker-frac", "0.5", "--max-eval", "3"], capture_output=True, text=True)
check(r.returncode == 0, "prepare_data.py runs: " + r.stderr[-300:])
meta = dataset.read_meta(out)
check(meta["classes"] == ["normal", "person", "landing_marker", "vehicle_wreck"] and meta["seq_len"] == 16, "meta.json")
tr, va, te = (dataset.load(out, s) for s in dataset.SPLITS)
check(tr["frames"].shape[1:] == (16, 96, 96) and tr["tel"].shape[1:] == (16, 16) and len(tr["y"]) == len(tr["frames"]), "array shapes")
check(len(va["y"]) == 3 * 12 and len(te["y"]) == 3 * 12 and sorted(set(va["tile"].tolist())) == list(range(12)),
      "val / test keep all twelve tiles of every sequence")
check(len(set(va["group"].tolist())) == 3, "one group per sequence")
used = lambda d, c: int(((d["y"] == c) & (d["w"] > 0)).sum())
check(used(tr, 1) > 50 and used(tr, 2) > 20 and used(tr, 0) > 50, "people, pasted markers and empty ground in train")
m = (tr["y"] == 1) & (tr["w"] > 0)
check(5 * 28 / 32 - 0.5 <= tr["size"][m].min() and tr["size"][m].max() <= 14 * 28 / 22 + 0.5, "people scaled to size_px for the altitude")
# a labelled person really is bright at the labelled position
n_ok = n_all = 0
for s, f in zip(*np.where(m)):
    px = np.clip(((tr["pos"][s, f] + 1) * 48).astype(int), 0, 95)
    n_all += 1; n_ok += tr["frames"][s, f, px[1], px[0]] > 120
check(n_all > 0 and n_ok / n_all > 0.9, f"labelled position lies on the object ({n_ok}/{n_all})")
check(meta["thermal"] and tr["thermal"].shape == tr["frames"].shape and (tr["tel"][..., 12] == 1).all(), "thermal channel stored for every tile frame")
n_hot = sum(tr["thermal"][s, f, px[1], px[0]] > 140
            for s, f in zip(*np.where(m)) for px in [np.clip(((tr["pos"][s, f] + 1) * 48).astype(int), 0, 95)])
bg = float((tr["thermal"][:, :, ::8, ::8] > 140).mean())
check(0.3 < n_hot / n_all < 0.98 and bg < 0.5 * n_hot / n_all,
      f"thermal is warm at many labelled people (a coarse sensor blurs small ones away), rarely elsewhere ({n_hot}/{n_all}, background {bg:.2f})")
out2 = os.path.join(tmp, "out2")
r = subprocess.run([sys.executable, os.path.join(HERE, "prepare_data.py"), "yolo", root, "--map",
                    os.path.join(HERE, "maps", "person_yolo.json"), "--out", out2, "--len", "16", "--no-thermal",
                    "--max-eval", "3"], capture_output=True, text=True)
d2 = dataset.load(out2, "train")
check(r.returncode == 0 and d2["thermal"] is None and (d2["tel"][..., 12] == 0).all() and not dataset.read_meta(out2)["thermal"],
      "--no-thermal: no thermal channel")
r = subprocess.run([sys.executable, os.path.join(HERE, "prepare_data.py"), "yolo", root, "--out", out, "--list-classes"],
                   capture_output=True, text=True)
check("'0'" in r.stdout and r.returncode == 0, "--list-classes prints the classes")

# ---------- WiSARD: camera + thermal pairs, not lined up ----------
# made-up pairs: thermal pixel t lies at camera pixel 2.0 * t + (300, 20)
K_, OX, OY = 2.0, 300, 20
wroot = os.path.join(tmp, "wisard")
def pair(cam_path, th_path, people, th_only=()):
    cam = rng.integers(60, 110, (1080, 1920)).astype(np.uint8)
    th = rng.integers(70, 90, (512, 640)).astype(np.uint8)
    for d, lab_people in ((cam_path, people), (th_path, list(people) + list(th_only))):
        os.makedirs(os.path.dirname(d), exist_ok=True)
        with open(os.path.splitext(d)[0] + ".txt", "w") as f:
            for (x, y) in lab_people:                       # person: 30 x 60 camera pixels around (x, y)
                tx, ty = (x - OX) / K_, (y - OY) / K_
                if d == cam_path:
                    f.write(f"0 {x / 1920} {y / 1080} {30 / 1920} {60 / 1080}\n")
                elif 0 <= tx < 640 and 0 <= ty < 512:
                    f.write(f"0 {tx / 640} {ty / 512} {15 / 640} {30 / 512}\n")
    for (x, y) in people:
        cam[y - 30:y + 30, x - 15:x + 15] = 240
    for (x, y) in list(people) + list(th_only):
        tx, ty = int((x - OX) / K_), int((y - OY) / K_)
        th[max(ty - 15, 0):ty + 15, max(tx - 7, 0):tx + 8] = 230
    cv2.imwrite(cam_path, cam); cv2.imwrite(th_path, th)
for i in range(14):
    ppl = [(int(rng.integers(400, 1500)), int(rng.integers(120, 950))) for _ in range(int(rng.integers(2, 4)))]
    pair(os.path.join(wroot, "flightA", "visual", f"{i:04d}.jpg"), os.path.join(wroot, "flightA", "thermal", f"{i:04d}.jpg"), ppl)
pair(os.path.join(wroot, "flightA", "visual", "0900.jpg"), os.path.join(wroot, "flightA", "thermal", "0900.jpg"), [])
pair(os.path.join(wroot, "flightB", "DJI_0001_W.jpg"), os.path.join(wroot, "flightB", "DJI_0001_T.jpg"),
     [(700, 500), (1200, 300)], th_only=[(1000, 800)])
pair(os.path.join(wroot, "flightB", "DJI_0002_W.jpg"), os.path.join(wroot, "flightB", "DJI_0002_T.jpg"), [(900, 600)])
check(pd.pair_key(os.path.join(wroot, "flightA", "visual", "0007.jpg"), wroot) ==
      pd.pair_key(os.path.join(wroot, "flightA", "thermal", "0007.jpg"), wroot) == "flighta_0007", "pair key drops the camera words")
items = pd.read_wisard(wroot)
check(len(items) == 17 and all("thermal" in it["thermal"]["path"] or it["thermal"]["path"].endswith("_T.jpg") for it in items),
      "every camera picture found its thermal picture (two naming styles)")
pd.register_pairs(items)
reg = np.array([it["reg"] for it in items], float)
check(np.abs(reg[:, 0] - K_).max() < 0.05 and np.abs(reg[:, 1] - OX).max() < 6 and np.abs(reg[:, 2] - OY).max() < 6,
      f"pairs lined up from the labels, also the one without people (worst {np.abs(reg - [K_, OX, OY]).max(0).round(2)})")
it = next(i for i in items if i["path"].endswith("DJI_0001_W.jpg"))
img = cv2.imread(it["path"], cv2.IMREAD_GRAYSCALE)
bx = [(1, b[1] * 1920 - 15, b[2] * 1080 - 30, 30, 60) for b in it["boxes"]]
cam_part, th_al, bx2 = pd.align_pair(img, bx, it, {"0": "person"}, {"person": 1})
check(cam_part.shape == th_al.shape and abs(cam_part.shape[1] - 1280) < 12 and abs(cam_part.shape[0] - 1024) < 12,
      "camera picture cut to the part the thermal picture covers")
check(len(bx2) == 3, "the person only the thermal camera saw is added")
check(all(th_al[int(b[2] + b[4] / 2), int(b[1] + b[3] / 2)] > 180 for b in bx2), "thermal picture is warm at every person after lining up")
check(sum(cam_part[int(b[2] + b[4] / 2), int(b[1] + b[3] / 2)] > 200 for b in bx2) == 2, "camera picture shows the two it can see")
wout = os.path.join(tmp, "wout")
r = subprocess.run([sys.executable, os.path.join(HERE, "prepare_data.py"), "wisard", wroot, "--map",
                    os.path.join(HERE, "maps", "wisard.json"), "--out", wout, "--len", "16", "--marker-frac", "0",
                    "--max-eval", "2"], capture_output=True, text=True)
check(r.returncode == 0, "prepare_data.py wisard runs: " + r.stderr[-400:])
dw = dataset.load(wout, "train")
mw = (dw["y"] == 1) & (dw["w"] > 0)
hits = [dw["thermal"][s_, f_, px[1], px[0]] > 140 for s_, f_ in zip(*np.where(mw))
        for px in [np.clip(((dw["pos"][s_, f_] + 1) * 48).astype(int), 0, 95)]]
bgw = float((dw["thermal"][:, :, ::8, ::8] > 140).mean())
check(dataset.read_meta(wout)["thermal"] and len(hits) > 100 and np.mean(hits) > 0.6 and bgw < 0.3 * np.mean(hits),
      f"real thermal ends up on the labelled people in the dataset ({np.mean(hits):.2f}, background {bgw:.2f})")

shutil.rmtree(tmp)
print("FAILED" if fails else "OK", f"({fails} failed)")
raise SystemExit(1 if fails else 0)
