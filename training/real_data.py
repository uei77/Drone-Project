"""Loads recorded flights (recorder.py) + labels (label_tool.py) as training streams.

A 320x240 flight becomes TWELVE streams, one per scan tile, because that is what the firmware
runs: the same model on every tile, each tile with its own memory. Every stream has the keys
synth.make_stream() returns:
  frames (L,96,96) uint8, tel (L,16) float32, y (L,), pos (L,2), has (L,), w (L,)
so the training and evaluation code does not care where the data came from.

What happens here, in order:
  1. frames are picked at the FLIGHT sweep gap (recordings are ~10 fps),
  2. telemetry features are built with telemetry.features() (same maths as firmware proto.c)
     using the real time gap between the picked frames,
  3. frame labels come from the time-based label file,
  4. each tile gets the situation only if the target centre is inside it; a tile the target
     only touches (centre up to EDGE_PX outside) gets weight 0; other tiles are "normal",
  5. frames with stale telemetry get weight 0 (the firmware makes no decision there either),
  6. a tile that a lost Wi-Fi packet touches (in this frame or the one before) gets weight 0.
Event frames whose target position is not known (no click on or around them) cannot be placed
in a tile: they get weight 0 everywhere.
"""
import glob, os
import numpy as np
from labels import LabelDoc, load_situations, labels_path
from telemetry import features, RAW_FIELDS
import tiles

TEL_STALE_MS = 500
EDGE_PX = 24          # target centre this close outside a tile: part of it is visible, weight 0
F = {n: i for i, n in enumerate(RAW_FIELDS)}


def pick_frames(esp_ms, rng, gap_ms=(150, 350)):
    """Indices of frames spaced like the flight build sees them: each next frame is the first
    one at least a random gap (uniform in gap_ms) after the previous. gap_ms should be the
    range of sweep times measured on the flight board (ESP32-S3: to be measured)."""
    esp_ms = np.asarray(esp_ms, np.int64)
    idx = [0]
    while True:
        nxt = np.searchsorted(esp_ms, esp_ms[idx[-1]] + rng.uniform(*gap_ms))
        if nxt >= len(esp_ms):
            break
        idx.append(int(max(nxt, idx[-1] + 1)))
    return np.array(idx)


def tile_has_hole(missing, k):
    """missing (n, n_chunks) bool -> (n,) bool: a lost packet covers rows of tile k.
    A packet is a run of bytes of the frame, so it covers whole rows across all columns."""
    n_chunks = missing.shape[1]
    per = tiles.CAM_W * tiles.CAM_H // n_chunks                     # bytes per packet
    first, last = np.arange(n_chunks) * per // tiles.CAM_W, (np.arange(n_chunks) * per + per - 1) // tiles.CAM_W
    y0 = tiles.ORIGINS[k][1]
    return missing[:, (last >= y0) & (first < y0 + tiles.TILE)].any(1)


def load_flight(npz_path, classes, version, rng, gap_ms=(150, 350), require_labels=True):
    d = np.load(npz_path)
    lp = labels_path(npz_path)
    if require_labels and not os.path.exists(lp):
        raise FileNotFoundError(f"{lp} missing. A flight with no events still needs an (empty) "
                                f"label file: open it once in label_tool.py.")
    doc = LabelDoc.load(lp, npz_path, version)
    if doc.situations_version != version:
        print(f"WARNING {os.path.basename(npz_path)}: labelled with situations v{doc.situations_version}, "
              f"current is v{version}")
    idx = pick_frames(d["esp_ms"], rng, gap_ms)
    t = d["esp_ms"][idx].astype(np.int64)
    raw = d["tel"][idx]
    dt_s = np.diff(t, prepend=t[0] - 200) / 1000.0        # first frame: typical gap, like firmware
    tel = features(raw[:, F["alt_m"]], raw[:, F["hdg_deg"]], raw[:, F["vz_mps"]], raw[:, F["vn_mps"]],
                   raw[:, F["ve_mps"]], raw[:, F["yawrate_dps"]], raw[:, F["wp_idx"]],
                   raw[:, F["wp_total"]], raw[:, F["wp_brg_deg"]], dt_s)
    y, pos, has, w = doc.frame_labels(t, classes)
    age = d["tel_age_ms"][idx].astype(np.int64)
    w[age > TEL_STALE_MS] = 0
    frames = d["frames"][idx]
    name = os.path.basename(npz_path)
    ev = np.full(len(t), -1)                               # which event a frame belongs to
    for k, e in enumerate(doc.events):
        ev[(t >= e["start_ms"]) & (t <= e["end_ms"])] = k
    holes = d["missing"][idx] if "missing" in d else None  # lost packets per frame (recorder.py)
    if frames.shape[1:] != (tiles.CAM_H, tiles.CAM_W):      # old 96x96 recording: one stream
        return [dict(frames=frames, tel=tel, y=y, pos=pos, has=has, w=w, esp_ms=t, name=name, tile=-1, ev=ev)]
    unplaced = (y > 0) & (has == 0)
    if unplaced.any():
        print(f"WARNING {name}: {int(unplaced.sum())} of {int((y > 0).sum())} event frames have no "
              f"target position (weight 0). Click the target at the start and the end of each event "
              f"in label_tool.py, and wherever it changes direction.")
    out = []
    lim = 1 + EDGE_PX / (tiles.TILE / 2)
    for k in range(tiles.SCAN_TILES):
        tx, ty = tiles.frame_to_tile(k, pos[:, 0], pos[:, 1])
        far = np.maximum(np.abs(tx), np.abs(ty))
        inside = (has > 0) & (far <= 1)
        edge = (has > 0) & (far > 1) & (far <= lim)
        yk = np.where(inside, y, 0)
        wk = w.copy()
        wk[edge | unplaced] = 0
        if holes is not None and holes.any():
            bad = tile_has_hole(holes, k)
            wk[bad | np.r_[False, bad[:-1]]] = 0   # the next frame uses this one as "previous image"
        pk = np.where(inside[:, None], np.stack([tx, ty], 1), 0).astype(np.float32)
        out.append(dict(frames=tiles.crop(frames, k), tel=tel, y=yk, pos=pk,
                        has=inside.astype(np.float32), w=wk, esp_ms=t, name=name, tile=k, ev=ev))
    return out


def load_flights(folder, situations=None, seed=0, gap_ms=(150, 350)):
    """All flights of a folder, split BY FLIGHT 70/15/15 (never by frame).
    Returns (train, val, test) lists of stream dicts and the class list."""
    version, classes = load_situations(situations)
    files = sorted(glob.glob(os.path.join(folder, "flight_*.npz")))
    if not files:
        raise FileNotFoundError(f"no flight_*.npz in {folder}")
    rng = np.random.default_rng(seed)
    flights = [load_flight(f, classes, version, rng, gap_ms) for f in files]
    order = np.random.default_rng(seed).permutation(len(flights))
    n_val = max(1, round(0.15 * len(flights))) if len(flights) >= 3 else 0
    n_test = n_val
    flat = lambda ids: [s for i in ids for s in flights[i]]          # all tile streams of a flight stay together
    return flat(order[n_test + n_val:]), flat(order[n_test:n_test + n_val]), flat(order[:n_test]), classes


def summary(streams, classes):
    """Per class: tile frames that count in training, and events (each event once, however
    many tiles it passes through; only events with at least one usable frame)."""
    out = {c["name"]: dict(frames=0, events=0) for c in classes}
    seen = set()
    for s in streams:
        use = s["w"] > 0
        for i, c in enumerate(classes):
            m = (s["y"] == i) & use
            out[c["name"]]["frames"] += int(m.sum())
            if i:
                seen |= {(c["name"], s["name"], int(e)) for e in np.unique(s["ev"][m])}
    for name, _, _ in seen:
        out[name]["events"] += 1
    return out


if __name__ == "__main__":
    import sys
    tr, va, te, classes = load_flights(sys.argv[1] if len(sys.argv) > 1 else "flights")
    for name, part in (("train", tr), ("val", va), ("test", te)):
        print(name, len({s["name"] for s in part}), "flights,", len(part), "tile streams", summary(part, classes))
