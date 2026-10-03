"""Turns a data source into a training dataset folder (layout: dataset.py).

usage:
  python prepare_data.py visdrone <root> --map maps/visdrone.json --out data/visdrone
  python prepare_data.py yolo     <root> --map maps/my_map.json   --out data/sard
  python prepare_data.py voc      <root> --map maps/heridal.json  --out data/heridal
  python prepare_data.py coco     <image folder> --ann instances.json --map ... --out ...
  python prepare_data.py wisard   <root>  --map maps/wisard.json  --out data/wisard
  python prepare_data.py flights  <folder with recorded flights>  --out data/flights
  python prepare_data.py synth                                     --out data/synth
  add --list-classes to an image source: prints the class names it found and stops.

Image datasets (photos with boxes) have no time and no telemetry, but the model looks at two
frames, telemetry and its own memory. So every photo becomes a short PRETEND FLIGHT:
  1. the photo is scaled until its objects are as small as they will be from 25-30 m
     (size_px in situations.json); a photo that gets smaller than the camera frame is repeated
     by mirroring, boxes included,
  2. a 320x240 window slides over it like the camera over the ground, at most half a tile per
     step (the firmware speed limit, scan.c), with exposure drift, blur and sensor noise,
  3. telemetry is computed from that motion (window moves up = drone flies forward),
  4. each of the twelve tiles becomes one stream; a tile is "person" when a person's centre is
     inside it, weight 0 when a target is only partly inside or too small to see,
  5. some flights get a landing marker pasted on the ground (no public dataset has one),
  6. a MADE-UP thermal frame is added (no public dataset of this kind has thermal): people
     are warm spots, so are some vehicles and some random spots with nobody there, and in some
     flights the ground is as warm as the people. The sensor resolution is varied between a
     160-pixel and a 32-pixel wide sensor. This teaches the model how to combine the two
     pictures; real thermal recordings are still needed to make it reliable. --no-thermal
     leaves the thermal channel out.
WiSARD has REAL thermal: camera and thermal pictures taken at the same moment, each with its
own YOLO labels, but not lined up with each other. `wisard` pairs the files by name, lines
each pair up with the help of the labelled people (see register_pairs), keeps the part of the
camera picture that the thermal picture covers, and uses the real thermal picture as the
third channel instead of the made-up one. People labelled only in the thermal picture (not
visible to the camera) count as people too.
The class map (--map) says which dataset class is which situation: {"pedestrian": "person",
"car": "vehicle_wreck", "ignored regions": "ignore"}; YOLO datasets use the class number as key:
{"0": "person"}. Classes not in the map are background.
"""
import argparse, glob, json, os, re, sys
import xml.etree.ElementTree as ET
from collections import Counter
import numpy as np
import dataset, tiles
from labels import load_situations
from telemetry import features

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")
VISDRONE = ["ignored regions", "pedestrian", "people", "bicycle", "car", "van", "truck", "tricycle",
            "awning-tricycle", "bus", "motor", "others"]
EDGE_PX = 24            # target centre this close outside a tile: partly visible, weight 0 (as real_data.py)
MIN_PX = 3.0            # smaller than this after scaling: cannot be seen, weight 0
REF_ALT_M = 28.0        # size_px in situations.json is for this altitude
MIN_CANVAS = (2 * tiles.CAM_W, 2 * tiles.CAM_H)


# ---------------------------------------------------------------- dataset readers
# every reader returns [dict(path=image file, boxes=[(class name, x, y, w, h)], norm=bool)]
# norm = True: box is (centre x, centre y, w, h) as a fraction of the image (YOLO)

def find_images(root):
    return sorted(p for p in glob.glob(os.path.join(root, "**", "*"), recursive=True)
                  if p.lower().endswith(IMG_EXT))


def read_yolo(root):
    items = []
    for img in find_images(root):
        txt = os.path.splitext(img)[0] + ".txt"
        parts = img.rsplit(os.sep + "images" + os.sep, 1)            # .../images/x.jpg -> .../labels/x.txt
        if len(parts) == 2:
            alt = os.path.splitext(parts[0] + os.sep + "labels" + os.sep + parts[1])[0] + ".txt"
            txt = alt if os.path.exists(alt) else txt
        boxes = []
        if os.path.exists(txt):
            for line in open(txt):
                v = line.split()
                if len(v) >= 5:
                    boxes.append((v[0],) + tuple(float(x) for x in v[1:5]))
        items.append(dict(path=img, boxes=boxes, norm=True))
    return items


def read_visdrone(root):
    index = {os.path.splitext(os.path.basename(p))[0]: p for p in find_images(root)}
    items = []
    for txt in sorted(glob.glob(os.path.join(root, "**", "annotations", "*.txt"), recursive=True)):
        img = index.get(os.path.splitext(os.path.basename(txt))[0])
        if not img:
            continue
        boxes = []
        for line in open(txt):
            v = [x for x in line.strip().split(",") if x != ""]
            if len(v) >= 6 and 0 <= int(v[5]) < len(VISDRONE):
                boxes.append((VISDRONE[int(v[5])],) + tuple(float(x) for x in v[:4]))
        items.append(dict(path=img, boxes=boxes, norm=False))
    return items


def read_coco(root, ann):
    with open(ann, encoding="utf-8") as f:
        j = json.load(f)
    index = {os.path.basename(p): p for p in find_images(root)}
    cat = {c["id"]: c["name"] for c in j["categories"]}
    by_img = {}
    for a in j["annotations"]:
        by_img.setdefault(a["image_id"], []).append((cat[a["category_id"]],) + tuple(float(x) for x in a["bbox"]))
    return [dict(path=index[os.path.basename(im["file_name"])], boxes=by_img.get(im["id"], []), norm=False)
            for im in j["images"] if os.path.basename(im["file_name"]) in index]


def read_voc(root):
    index = {os.path.splitext(os.path.basename(p))[0]: p for p in find_images(root)}
    items = []
    for xml in sorted(glob.glob(os.path.join(root, "**", "*.xml"), recursive=True)):
        img = index.get(os.path.splitext(os.path.basename(xml))[0])
        if not img:
            continue
        boxes = []
        for o in ET.parse(xml).getroot().iter("object"):
            b = o.find("bndbox")
            x0, y0, x1, y1 = (float(b.find(k).text) for k in ("xmin", "ymin", "xmax", "ymax"))
            boxes.append((o.find("name").text.strip(), x0, y0, x1 - x0, y1 - y0))
        items.append(dict(path=img, boxes=boxes, norm=False))
    return items


# ---------------------------------------------------------------- WiSARD: camera + thermal pairs

THERMAL_PATH = re.compile(r"therm|lwir|infrared|(?<![a-z])ir(?![a-z])|_t$")
CAMERA_WORDS = re.compile(r"thermal|therm|lwir|infrared|visual|visible|rgb|(?<![a-z])(vis|eo|ir)(?![a-z])|(?<=_)[twvz]$")


def pair_key(path, root):
    """The file's path without the words that name the camera: both pictures of a pair get
    the same key (flight1/visual/0007.jpg and flight1/thermal/0007.jpg -> flight1_0007)."""
    rel = os.path.splitext(os.path.relpath(path, root))[0].lower().replace(os.sep, "/")
    return re.sub(r"[^a-z0-9]+", "_", CAMERA_WORDS.sub("", rel)).strip("_")


def read_wisard(root):
    """Camera pictures of the pairs, each with it["thermal"] = its thermal picture (path, boxes)."""
    cam, th = {}, {}
    for it in read_yolo(root):
        rel = os.path.splitext(os.path.relpath(it["path"], root))[0].lower().replace(os.sep, "/")
        (th if THERMAL_PATH.search(rel) else cam)[pair_key(it["path"], root)] = it
    both = sorted(set(cam) & set(th))
    how = "by file name"
    if not both and cam and len(cam) == len(th):          # names differ: same order in both lists
        both, how = list(range(len(cam))), "by position in the sorted file lists (names did not match)"
        cam, th = dict(enumerate(cam[k] for k in sorted(cam))), dict(enumerate(th[k] for k in sorted(th)))
    print(f"{len(cam)} camera and {len(th)} thermal pictures, {len(both)} pairs {how}")
    if not both:
        ex = lambda d: [os.path.relpath(v["path"], root) for v in list(d.values())[:3]]
        sys.exit("could not pair camera and thermal pictures. A thermal picture needs 'thermal', 'lwir' or 'ir' "
                 f"in its path, and the rest of the name must match.\n  camera examples:  {ex(cam)}\n  thermal examples: {ex(th)}")
    for k in both[:3]:
        print("  pair:", os.path.relpath(cam[k]["path"], root), "<->", os.path.relpath(th[k]["path"], root))
    return [dict(cam[k], thermal=th[k]) for k in both]


def register_pairs(items):
    """Finds for every pair how the thermal picture lies in the camera picture:
    camera pixel = k * thermal pixel + (ox, oy). Sets it["reg"] = (k, ox, oy).
    The two cameras see the same people, so the labels give it away:
      k  (camera pixels per thermal pixel) from the distances between people, in pictures with
         the same two or more people in both; the median over a flight (one folder) is used,
      ox, oy from the people of this pair (the shift that puts most of them on top of each
         other); for a pair without people, or when in doubt, the median of its flight.
    Flat ground far below is assumed (no rotation, same scale in x and y)."""
    from PIL import Image

    def centres(it):
        w, h = Image.open(it["path"]).size
        return np.array([(b[1] * w, b[2] * h) for b in it["boxes"]], float).reshape(-1, 2), w, h

    info, ks = [], {}
    for it in items:
        V, w, h = centres(it)
        T, wt, ht = centres(it["thermal"])
        flight = os.path.dirname(it["path"])
        info.append((it, V, T, w, h, wt, ht, flight))
        if len(V) == len(T) >= 2:
            V2, T2 = V[np.argsort(V[:, 0])], T[np.argsort(T[:, 0])]          # same people, left to right
            dv, dt = np.linalg.norm(V2[1:] - V2[:-1], axis=1), np.linalg.norm(T2[1:] - T2[:-1], axis=1)
            ok = dt > 20
            if ok.any():
                ks.setdefault(flight, []).append(float(np.median(dv[ok] / dt[ok])))
    all_k = [k for v in ks.values() for k in v]
    if not all_k:
        print("WARNING: no pair shows two or more people in both pictures, so the size of the thermal picture\n"
              "         cannot be measured: assuming it covers the middle 60 % of the camera picture")
    # shift: every (camera person, thermal person) couple proposes one; the best proposal puts
    # the most people on top of each other (the two pictures often show different people)
    cand, offs = [], {}
    for it, V, T, w, h, wt, ht, flight in info:
        k = float(np.median(ks[flight])) if flight in ks else float(np.median(all_k)) if all_k else 0.6 * w / wt
        it["reg"] = [k, None, None]
        props = []
        for v in V:
            for t in T:
                d = np.linalg.norm(V[:, None] - (k * T + (v - k * t))[None], axis=2)      # camera x thermal
                hit = d.min(1) < max(20, 0.02 * w)
                o = (V[hit] - k * T[d.argmin(1)[hit]]).mean(0)
                props.append((int(hit.sum()), o))
        props.sort(key=lambda p: -p[0])
        cand.append(props)
        if props and (props[0][0] >= 2 or len(V) == len(T) == 1):                          # no doubt
            it["reg"][1:] = props[0][1]
            offs.setdefault(flight, []).append(props[0][1])
    n_own = 0
    for (it, V, T, w, h, wt, ht, flight), props in zip(info, cand):
        k = it["reg"][0]
        if it["reg"][1] is not None:
            n_own += 1
        elif flight not in offs:
            it["reg"][1:] = props[0][1] if props else ((w - k * wt) / 2, (h - k * ht) / 2)
        else:                              # one person matched at best: the proposal nearest to the flight's shift
            med = np.median(offs[flight], 0)
            best = min((p[1] for p in props if p[0] == props[0][0]), key=lambda o: np.linalg.norm(o - med), default=med)
            it["reg"][1:] = best if np.linalg.norm(best - med) < 0.1 * w else med
    print(f"lined up: {n_own} pairs by their own people, {len(items) - n_own} by their flight; "
          f"camera pixels per thermal pixel {np.median([it['reg'][0] for it in items]):.2f} (median)")


def align_pair(img, boxes, it, cmap, idx):
    """Cuts the camera picture down to the part the thermal picture covers and brings the
    thermal picture onto it. Returns (camera part, thermal on the same grid, boxes) or None."""
    import cv2
    th = cv2.imread(it["thermal"]["path"], cv2.IMREAD_GRAYSCALE)
    if th is None:
        return None
    h, w = img.shape
    ht, wt = th.shape
    k, ox, oy = it["reg"]
    x0, y0 = max(0, int(np.ceil(ox))), max(0, int(np.ceil(oy)))
    x1, y1 = min(w, int(ox + k * wt)), min(h, int(oy + k * ht))
    if x1 - x0 < 200 or y1 - y0 < 200:
        return None
    M = np.array([[k, 0, ox - x0], [0, k, oy - y0]], np.float32)
    th = cv2.warpAffine(th, M, (x1 - x0, y1 - y0), flags=cv2.INTER_AREA if k < 1 else cv2.INTER_LINEAR)
    # people the thermal camera saw and the camera did not: mapped into the camera picture
    for b in it["thermal"]["boxes"]:
        if b[0] in cmap:
            bw, bh = b[3] * wt * k, b[4] * ht * k
            cx, cy = b[1] * wt * k + ox, b[2] * ht * k + oy
            if not any(abs(cx - (v[1] + v[3] / 2)) < max(bw, v[3]) and abs(cy - (v[2] + v[4] / 2)) < max(bh, v[4])
                       for v in boxes):
                boxes.append((idx[cmap[b[0]]], cx - bw / 2, cy - bh / 2, bw, bh))
    boxes = [(c, x - x0, y - y0, bw, bh) for c, x, y, bw, bh in boxes
             if x0 <= x + bw / 2 < x1 and y0 <= y + bh / 2 < y1]
    return img[y0:y1, x0:x1], th, boxes


# ---------------------------------------------------------------- pretend flight from one photo

def mirror_copies(x, size, n, pad, total):
    """Start positions of a box [x, x+size) of an n-pixel image inside a symmetric-padded axis."""
    out = []
    for k in range(-(pad // (2 * n)) - 1, (total // (2 * n)) + 2):
        for start in (pad + 2 * n * k + x, pad + 2 * n * k + 2 * n - x - size):
            if start + size > 0 and start < total:
                out.append(start)
    return out


def scale_pad(img, s):
    """Picture scaled by s, then mirrored outwards until it is big enough to fly over.
    Returns (canvas float32, scaled width, scaled height, left pad, top pad)."""
    import cv2
    h, w = img.shape
    sw, sh = max(1, round(w * s)), max(1, round(h * s))
    small = cv2.resize(img, (sw, sh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    cw, ch = max(sw, MIN_CANVAS[0]), max(sh, MIN_CANVAS[1])
    left, top = (cw - sw) // 2, (ch - sh) // 2
    canvas = np.pad(small, ((top, ch - sh - top), (left, cw - sw - left)), mode="symmetric")
    return canvas.astype(np.float32), sw, sh, left, top


def make_canvas(img, boxes, s):
    """scale_pad() of a photo and its boxes:
    boxes (cls, x, y, w, h) in photo pixels -> array rows (cls, x, y, w, h) in canvas pixels."""
    canvas, sw, sh, left, top = scale_pad(img, s)
    ch, cw = canvas.shape
    out = []
    for c, x, y, bw, bh in boxes:
        x, y, bw, bh = x * s, y * s, bw * s, bh * s
        for xx in mirror_copies(x, bw, sw, left, cw):
            for yy in mirror_copies(y, bh, sh, top, ch):
                out.append((c, xx, yy, bw, bh))
    return canvas, np.array(out, np.float32).reshape(-1, 5)


def paste_marker(canvas, cx, cy, size, rng):
    """Landing marker (an H in a square, any rotation) drawn on the ground at (cx, cy)."""
    import cv2
    n = int(size)
    m = int(np.ceil(n * 1.5))
    off = (m - n) // 2
    bg, fg = rng.uniform(190, 250), rng.uniform(15, 60)
    if rng.random() < 0.2:
        bg, fg = fg, bg                                  # dark square, bright H
    patch = np.zeros((m, m), np.float32); alpha = np.zeros((m, m), np.float32)
    patch[off:off + n, off:off + n] = bg; alpha[off:off + n, off:off + n] = 1
    b = max(2, n // 6)
    for xa in (off + b, off + n - 2 * b):
        patch[off + b:off + n - b, xa:xa + b] = fg
    patch[off + n // 2 - b // 2:off + n // 2 - b // 2 + b, off + b:off + n - b] = fg
    rot = cv2.getRotationMatrix2D((m / 2, m / 2), rng.uniform(0, 360), 1.0)
    patch, alpha = cv2.warpAffine(patch, rot, (m, m)), cv2.warpAffine(alpha, rot, (m, m))
    x0, y0 = int(cx - m / 2), int(cy - m / 2)
    if x0 < 0 or y0 < 0 or x0 + m > canvas.shape[1] or y0 + m > canvas.shape[0]:
        return None
    reg = canvas[y0:y0 + m, x0:x0 + m]
    reg[:] = reg * (1 - alpha) + patch * alpha
    return (cx - n / 2, cy - n / 2, n, n)


def thermal_field(canvas, objs, person, vehicle, rng):
    """Made-up ground temperature (degrees above an arbitrary level) for a canvas, on the
    thermal grid (half the camera resolution), as a low-resolution sensor would see it."""
    import cv2
    ch, cw = canvas.shape
    h2, w2 = (ch + 1) // 2, (cw + 1) // 2
    f = cv2.GaussianBlur(rng.normal(0, 1, (h2, w2)).astype(np.float32), (0, 0), rng.uniform(8, 30))
    f *= rng.uniform(0.5, 3.0) / (f.std() + 1e-6)                      # slow changes of the ground
    vis = cv2.GaussianBlur(cv2.resize(canvas, (w2, h2), interpolation=cv2.INTER_AREA), (0, 0), 3)
    f += rng.uniform(-0.03, 0.05) * (vis - vis.mean())                 # bright ground a bit warmer, or not
    yy, xx = np.mgrid[0:h2, 0:w2].astype(np.float32)

    def blob(cx, cy, sx, sy, dt):
        f[:] += dt * np.exp(-0.5 * (((xx - cx / 2) / max(sx / 2, 0.7)) ** 2 + ((yy - cy / 2) / max(sy / 2, 0.7)) ** 2))

    stand_out = rng.random() > 0.15           # else: ground as warm as the people (hot day)
    for c, x, y, w, h in objs:
        if c == person and stand_out:
            blob(x + w / 2, y + h / 2, w / 2.5, h / 2.5, rng.uniform(3, 15))
        elif c == vehicle and rng.random() < 0.5:
            blob(x + w / 2, y + h / 2, w / 3, h / 3, rng.uniform(5, 25))
    for _ in range(int(rng.integers(0, 7))):  # warm things that are not a target: rocks, animals, fires
        blob(rng.uniform(0, cw), rng.uniform(0, ch), rng.uniform(2, 20), rng.uniform(2, 20), rng.uniform(3, 15))
    return sensor_view(f, cw, ch, rng)


def sensor_view(f, cw, ch, rng):
    """A temperature picture on the thermal grid as a small, noisy sensor would deliver it."""
    import cv2
    h2, w2 = f.shape
    r = rng.uniform(2, 9)                     # camera pixels per sensor pixel: 160 .. 35 pixels wide
    small = cv2.resize(f, (max(1, round(cw / r)), max(1, round(ch / r))), interpolation=cv2.INTER_AREA)
    small = small + rng.normal(0, rng.uniform(0.05, 0.3), small.shape).astype(np.float32)
    f = cv2.resize(small, (w2, h2), interpolation=cv2.INTER_LINEAR)
    return np.roll(f, rng.integers(-2, 3, 2), (0, 1))                  # the two cameras are never lined up exactly


def thermal_u8(t):
    """Temperature frame -> model input: 128 = median of the frame, 8 counts per degree."""
    return np.clip(128 + (t - np.median(t)) * 8, 0, 255).astype(np.uint8)


def fly(cw, ch, steps, rng):
    """Top-left corner of the camera frame at every step: straight line, bounces at the border."""
    lim = np.array([cw - tiles.CAM_W, ch - tiles.CAM_H], float)
    pos = rng.uniform(0, 1, 2) * lim
    speed = 0.0 if rng.random() < 0.15 else rng.uniform(0, tiles.TILE / 2)     # some flights hover
    ang = rng.uniform(0, 2 * np.pi)
    v = speed * np.array([np.cos(ang), np.sin(ang)])
    out = []
    for _ in range(steps):
        out.append(np.rint(pos).astype(int))
        pos = pos + v + rng.normal(0, 1.0, 2)             # vibration
        for a in range(2):
            if pos[a] < 0:
                pos[a], v[a] = -pos[a], -v[a]
            if pos[a] > lim[a]:
                pos[a], v[a] = 2 * lim[a] - pos[a], -v[a]
        pos = np.clip(pos, 0, lim)
    return np.array(out)


def flight_telemetry(path, alt, gap_ms, rng):
    """Telemetry that matches the window motion. Image top = drone nose (CAM_MOUNT_DOWN):
    the window moving up over the ground is the drone flying forward."""
    n = len(path)
    dt = rng.uniform(*gap_ms, n) / 1000.0
    d = np.diff(path, axis=0, prepend=path[:1]).astype(float)
    d[0] = d[1] if n > 1 else 0
    gsd = tiles.metres_per_px(alt)
    v_fwd, v_rgt = -d[:, 1] * gsd / dt, d[:, 0] * gsd / dt
    hdg = rng.uniform(0, 360)
    c, s = np.cos(np.deg2rad(hdg)), np.sin(np.deg2rad(hdg))
    total = int(rng.integers(4, 30))
    tel = features(alt + rng.normal(0, 0.3, n), np.full(n, hdg), rng.normal(0, 0.2, n),
                   v_fwd * c - v_rgt * s, v_fwd * s + v_rgt * c, rng.normal(0, 3, n),
                   np.full(n, rng.integers(0, total)), np.full(n, total),
                   np.full(n, hdg + rng.normal(0, 20)), np.r_[0.2, dt[1:]])
    return tel, dt * 1000.0


def tile_labels(objs, ox, oy, prio):
    """objs rows (cls, x, y, w, h) in canvas px, cls -1 = ignore region. Frame corner (ox, oy).
    Returns y, pos, has, w, size for the twelve tiles."""
    T = tiles.TILE
    y = np.zeros(tiles.SCAN_TILES, np.int64); pos = np.zeros((tiles.SCAN_TILES, 2), np.float32)
    has = np.zeros(tiles.SCAN_TILES, np.float32); w = np.ones(tiles.SCAN_TILES, np.float32)
    size = np.zeros(tiles.SCAN_TILES, np.float32)
    if not len(objs):
        return y, pos, has, w, size
    cls = objs[:, 0].astype(int)
    bx, by, bw, bh = objs[:, 1] - ox, objs[:, 2] - oy, objs[:, 3], objs[:, 4]
    sz = np.sqrt(bw * bh)
    usable = (cls > 0) & (sz >= MIN_PX) & (np.maximum(bw, bh) <= T)
    for k, (x0, y0) in enumerate(tiles.ORIGINS):
        lx, ly = bx + bw / 2 - x0, by + bh / 2 - y0
        inside = (lx >= 0) & (lx < T) & (ly >= 0) & (ly < T)
        near = (lx > -EDGE_PX) & (lx < T + EDGE_PX) & (ly > -EDGE_PX) & (ly < T + EDGE_PX)
        overlap = (bx < x0 + T) & (bx + bw > x0) & (by < y0 + T) & (by + bh > y0)
        if (overlap & (cls < 0)).any():
            w[k] = 0
        cand = np.where(inside & usable)[0]
        if len(cand):
            best = max(cand, key=lambda i: (prio[cls[i]], -np.hypot(lx[i] - T / 2, ly[i] - T / 2)))
            y[k], has[k], size[k] = cls[best], 1, sz[best]
            pos[k] = lx[best] / (T / 2) - 1, ly[best] / (T / 2) - 1
        elif (near & (cls > 0)).any():
            w[k] = 0                                     # only partly in the tile, or too small / too big
    return y, pos, has, w, size


def pretend_flight(img, boxes, s, classes, a, rng, marker, thermal_img=None):
    """One photo -> twelve tile streams of a.len frames. None if nothing useful comes out.
    thermal_img: a real thermal picture on the grid of img (WiSARD); else one is made up."""
    import cv2
    alt = rng.uniform(22, 32)
    s = s * REF_ALT_M / alt                              # higher = smaller on the ground
    if img.shape[1] * s < tiles.TILE or img.shape[0] * s < tiles.TILE:
        return None                                      # would need dozens of mirror copies
    canvas, objs = make_canvas(img, boxes, s)
    ch, cw = canvas.shape
    path = fly(cw, ch, a.len, rng)
    if marker is not None:
        lo, hi = classes[marker]["size_px"]
        fx, fy = path[rng.integers(0, len(path))] + rng.uniform(0.15, 0.85, 2) * (tiles.CAM_W, tiles.CAM_H)
        box = paste_marker(canvas, fx, fy, rng.uniform(lo, hi) * REF_ALT_M / alt, rng)
        if box:
            objs = np.vstack([objs, np.array([(marker,) + box], np.float32)])
    blur = rng.uniform(0, 0.9)
    if blur > 0.3:
        canvas = cv2.GaussianBlur(canvas, (0, 0), blur)
    tel, dt_ms = flight_telemetry(path, alt, a.gap_ms, rng)
    names = [c["name"] for c in classes]
    if thermal_img is not None:          # 8-bit picture, brighter = warmer: used as counts around its median
        tcan = cv2.resize(scale_pad(thermal_img, s)[0], ((cw + 1) // 2, (ch + 1) // 2), interpolation=cv2.INTER_AREA)
        field = sensor_view(tcan * (-1 if a.black_hot else 1) / 8, cw, ch, rng)
    elif a.no_thermal:
        field = None
    else:
        field = thermal_field(canvas, objs, names.index("person") if "person" in names else -9,
                              names.index("vehicle_wreck") if "vehicle_wreck" in names else -9, rng)
    th = np.zeros((a.len, tiles.CAM_H // tiles.THERMAL_DIV, tiles.CAM_W // tiles.THERMAL_DIV), np.uint8)
    prio = [c.get("priority", 0) for c in classes]
    gain, offset, noise = rng.uniform(0.8, 1.2), rng.uniform(-15, 15), rng.uniform(1.5, 5)
    frames = np.zeros((a.len, tiles.CAM_H, tiles.CAM_W), np.uint8)
    lab = []
    for i, (ox, oy) in enumerate(path):
        gain = float(np.clip(gain + rng.normal(0, 0.01), 0.7, 1.3))           # exposure drift
        f = canvas[oy:oy + tiles.CAM_H, ox:ox + tiles.CAM_W] * gain + offset
        frames[i] = np.clip(f + rng.normal(0, noise, f.shape), 0, 255)
        lab.append(tile_labels(objs, ox, oy, prio))
        if field is not None:
            th[i] = thermal_u8(field[oy // 2:oy // 2 + th.shape[1], ox // 2:ox // 2 + th.shape[2]])
    y, pos, has, w, size = (np.stack(x, 1) for x in zip(*lab))              # (12, len, ...)
    streams = [dict(frames=tiles.crop(frames, k), tel=tel, y=y[k], pos=pos[k], has=has[k], w=w[k],
                    size=size[k], dt_ms=dt_ms, tile=k) for k in range(tiles.SCAN_TILES)]
    if field is not None:
        tel[:, 12] = 1.0                                 # "thermal frame present"
        for k, st in enumerate(streams):
            st["thermal"] = tiles.thermal_crop(th, k)
    return streams


# ---------------------------------------------------------------- sources

def from_images(a, classes):
    import cv2
    readers = dict(yolo=read_yolo, visdrone=read_visdrone, voc=read_voc, wisard=read_wisard,
                   coco=lambda r: read_coco(r, a.ann))
    items = readers[a.source](a.path)
    found = Counter(b[0] for it in items for b in it["boxes"])
    print(f"{len(items)} images, classes found:", dict(found.most_common()))
    if a.list_classes or not items:
        return None
    if not a.map:
        sys.exit("--map is needed: which dataset class is which situation (see maps/)")
    with open(a.map, encoding="utf-8") as f:
        cmap = {k: v for k, v in json.load(f).items() if not k.startswith("_")}
    idx = {c["name"]: i for i, c in enumerate(classes)}
    idx["ignore"] = -1
    bad = [v for v in cmap.values() if v not in idx]
    if bad:
        sys.exit(f"--map names unknown situations {sorted(set(bad))}; allowed: {sorted(idx)}")
    if not set(cmap) & set(found):
        sys.exit(f"no class of --map is in the dataset. Map keys: {sorted(cmap)}; dataset: {sorted(found)}")
    marker = idx.get("landing_marker") if a.marker_frac > 0 else None
    if a.source == "wisard":
        register_pairs(items)

    rng = np.random.default_rng(a.seed)
    items = [items[i] for i in rng.permutation(len(items))][:a.max_images]
    n_eval = min(a.max_eval, max(1, len(items) // 10))
    split_of = ["val"] * n_eval + ["test"] * n_eval + ["train"] * (len(items) - 2 * n_eval)
    writers = {s: dataset.Writer(os.path.join(a.out, s), a.len) for s in dataset.SPLITS}
    scales, skipped = [], Counter()
    for n, (it, split) in enumerate(zip(items, split_of)):
        img = cv2.imread(it["path"], cv2.IMREAD_GRAYSCALE)
        if img is None:
            skipped["unreadable"] += 1; continue
        h, w = img.shape
        boxes = [(idx[cmap[b[0]]],) + ((b[1] - b[3] / 2) * w, (b[2] - b[4] / 2) * h, b[3] * w, b[4] * h)
                 if it["norm"] else (idx[cmap[b[0]]],) + tuple(b[1:])
                 for b in it["boxes"] if b[0] in cmap]
        th_img = None
        if "thermal" in it:
            al = align_pair(img, boxes, it, cmap, idx)
            if al is None:
                skipped["thermal picture unreadable or beside the camera picture"] += 1; continue
            img, th_img, boxes = al
        # scale: the first situation in the photo decides how small everything becomes
        ref = min((b[0] for b in boxes if b[0] > 0), default=0)
        if ref:
            px = np.median([np.sqrt(b[3] * b[4]) for b in boxes if b[0] == ref])
            s = rng.uniform(*classes[ref]["size_px"]) / max(px, 1.0)
            scales.append(s)
        else:
            s = float(np.median(scales)) if scales else a.default_scale     # photo without objects
        s = min(s, 2.0)                                                     # never blow a photo up much
        streams = pretend_flight(img, boxes, s, classes, a, rng,
                                 marker if rng.random() < a.marker_frac else None, th_img)
        if streams is None:
            skipped["too small after scaling"] += 1; continue
        if split == "train":                 # all tiles with a target + a few empty ones
            hit = [k for k, st in enumerate(streams) if st["y"].any()]
            rest = [k for k in range(len(streams)) if k not in hit]
            keep = hit + list(rng.permutation(rest)[:a.neg_tiles])
            streams = [streams[k] for k in sorted(keep)]
        for st in streams:                   # val / test keep all twelve: sweep-level evaluation
            writers[split].add(st, n)
        if (n + 1) % 100 == 0:
            print(f"  {n + 1}/{len(items)} images", flush=True)
    if skipped:
        print("skipped:", dict(skipped))
    if scales:
        print(f"photos were scaled by {np.median(scales):.2f} (median), {min(scales):.2f} .. {max(scales):.2f}")
    return writers, dict(tiles=tiles.SCAN_TILES, gap_ms=list(a.gap_ms), thermal=not a.no_thermal)


def from_flights(a, classes):
    import real_data
    tr, va, te, _ = real_data.load_flights(a.path, seed=a.seed, gap_ms=tuple(a.gap_ms))
    writers = {s: dataset.Writer(os.path.join(a.out, s), a.len) for s in dataset.SPLITS}
    for split, part in (("train", tr), ("val", va), ("test", te)):
        for st in part:
            st["dt_ms"] = np.diff(st["esp_ms"], prepend=st["esp_ms"][0] - 200).astype(np.float32)
            writers[split].add(st, st["name"])
    return writers, dict(tiles=tiles.SCAN_TILES, gap_ms=list(a.gap_ms))


def from_synth(a):
    import synth
    writers = {s: dataset.Writer(os.path.join(a.out, s), 600) for s in dataset.SPLITS}
    for split, seed0, n, length in (("train", 1000, 64, 600), ("val", 5000, 8, 600), ("test", 9000, 8, 3000)):
        for i in range(n):
            st = synth.make_stream(seed0 + i, length)
            st["dt_ms"] = st["tel"][:, 8] * 3 * (1000 / 15.0)      # one synthetic time step = 66.7 ms
            writers[split].add(st, i)
    return writers, dict(tiles=1, stage1_merge=[1, 2, 3])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("source", choices=["yolo", "coco", "visdrone", "voc", "wisard", "flights", "synth"])
    ap.add_argument("path", nargs="?", help="dataset root / flights folder")
    ap.add_argument("--out", required=True, help="dataset folder to create, e.g. data/visdrone")
    ap.add_argument("--map", help="JSON: dataset class -> situation name or 'ignore' (image sources)")
    ap.add_argument("--ann", help="COCO annotation file")
    ap.add_argument("--situations", default=None, help="default: situations.json next to this file")
    ap.add_argument("--list-classes", action="store_true")
    ap.add_argument("--len", type=int, default=32, help="frames per stream")
    ap.add_argument("--gap-ms", type=float, nargs=2, default=(150, 350),
                    help="sweep time range of the flight board in ms (measure it: 'sweep ... ms' in the log)")
    ap.add_argument("--max-images", type=int, default=1500)
    ap.add_argument("--max-eval", type=int, default=150, help="most images in val and in test")
    ap.add_argument("--neg-tiles", type=int, default=2, help="empty tiles kept per training image")
    ap.add_argument("--marker-frac", type=float, default=0.15, help="share of flights with a pasted landing marker")
    ap.add_argument("--no-thermal", action="store_true", help="image sources: do not add the made-up thermal channel")
    ap.add_argument("--black-hot", action="store_true", help="wisard: thermal pictures show warm as dark")
    ap.add_argument("--default-scale", type=float, default=0.25, help="scale for photos without objects, until one with objects was seen")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    if a.source != "synth" and not a.path:
        ap.error("path is needed")

    if a.source == "synth":
        import synth
        names = synth.NAMES
        res = from_synth(a)
    else:
        _, classes = load_situations(a.situations)
        names = [c["name"] for c in classes]
        res = from_flights(a, classes) if a.source == "flights" else from_images(a, classes)
    if res is None:
        return
    writers, extra = res
    for split, wr in writers.items():
        n = wr.close()
        d = dataset.load(a.out, split)
        use = d["w"] > 0
        per = {names[c]: int(((d["y"] == c) & use).sum()) for c in range(len(names))}
        print(f"{split:5s} {n:5d} streams, {len(set(d['group'].tolist())):4d} sequences, usable frames per class {per}")
    dataset.write_meta(a.out, names, f"{a.source}:{a.path or ''}", a.len if a.source != "synth" else 600, **extra)
    size = sum(os.path.getsize(os.path.join(a.out, s, f)) for s in dataset.SPLITS for f in ("frames.u8", "thermal.u8")
               if os.path.exists(os.path.join(a.out, s, f)))
    print(f"wrote {a.out} ({size / 1e9:.2f} GB). Next: python train.py --data {a.out}")


if __name__ == "__main__":
    main()
