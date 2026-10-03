"""Training data on disk: one folder per dataset, the same layout whatever the source
(downloaded image dataset, recorded flights, synthetic streams). Written by prepare_data.py,
read by train.py and evaluate.py.

  <dataset>/meta.json                 classes, sequence length, where the data came from
  <dataset>/<train|val|test>/frames.u8    uint8, (N, L, 96, 96): N streams of L frames
  <dataset>/<train|val|test>/thermal.u8   uint8, (N, L, 96, 96): the thermal channel of the same
                                          tiles (only when the source has thermal; see models.py)
  <dataset>/<train|val|test>/arrays.npz   per frame: tel (N,L,16), y, pos (N,L,2), has, w,
                                          size (target size in px, 0 = none), dt_ms
                                          per stream: group (streams of one sweep sequence share
                                          it), tile (0..11, -1 = whole-frame stream)
A stream is what ONE tile sees over time; frames.u8 is memory-mapped, so a dataset larger than
the RAM still trains.
"""
import json, os
import numpy as np

SPLITS = ("train", "val", "test")
PER_FRAME = ("tel", "y", "pos", "has", "w", "size", "dt_ms")


class Writer:
    def __init__(self, folder, seq_len):
        os.makedirs(folder, exist_ok=True)
        self.folder, self.L = folder, seq_len
        self.f = open(os.path.join(folder, "frames.u8"), "wb")
        self.ft = None                    # thermal.u8, opened by the first stream that has thermal
        self.rows = {k: [] for k in PER_FRAME + ("group", "tile")}
        self.groups = {}

    def add(self, s, group):
        """s: one stream (frames (n,96,96) uint8, tel, y, pos, has, w; optional size, dt_ms, tile,
        thermal (n,96,96) uint8: then every stream of the dataset must have it).
        It is cut into pieces of seq_len frames; a last piece shorter than half is dropped, a
        longer one is filled up with weight-0 frames. `group` names the sweep sequence."""
        n, L = len(s["y"]), self.L
        full = dict(s, size=s.get("size", np.zeros(n, np.float32)), dt_ms=s.get("dt_ms", np.full(n, 200.0)))
        for seg, a in enumerate(range(0, n, L)):
            m = min(L, n - a)
            if m < L // 2:
                break
            fr = np.zeros((L, 96, 96), np.uint8)
            fr[:m] = s["frames"][a:a + m]
            self.f.write(fr.tobytes())
            if "thermal" in s:
                self.ft = self.ft or open(os.path.join(self.folder, "thermal.u8"), "wb")
                fr[:m] = s["thermal"][a:a + m]
                self.ft.write(fr.tobytes())
            for k in PER_FRAME:
                v = np.asarray(full[k][a:a + m])
                pad = np.zeros((L,) + v.shape[1:], np.float32 if k != "y" else np.int64)
                pad[:m] = v
                self.rows[k].append(pad)
            self.rows["group"].append(self.groups.setdefault((group, seg), len(self.groups)))
            self.rows["tile"].append(s.get("tile", -1))

    def close(self):
        self.f.close()
        if self.ft:
            self.ft.close()
        np.savez(os.path.join(self.folder, "arrays.npz"), **{k: np.array(v) for k, v in self.rows.items()})
        return len(self.rows["tile"])


def write_meta(root, classes, source, seq_len, **extra):
    with open(os.path.join(root, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(dict(classes=list(classes), source=source, seq_len=seq_len, **extra), f, indent=1)


def read_meta(root):
    with open(os.path.join(root, "meta.json"), encoding="utf-8") as f:
        return json.load(f)


def load(root, split):
    """dict of arrays for one split; frames (and thermal, None if the dataset has none) are
    read-only memory maps."""
    folder = os.path.join(root, split)
    d = dict(np.load(os.path.join(folder, "arrays.npz")))
    n, L = d["y"].shape
    d["frames"] = (np.memmap(os.path.join(folder, "frames.u8"), np.uint8, "r", shape=(n, L, 96, 96))
                   if n else np.zeros((0, L, 96, 96), np.uint8))
    tp = os.path.join(folder, "thermal.u8")
    d["thermal"] = np.memmap(tp, np.uint8, "r", shape=(n, L, 96, 96)) if n and os.path.exists(tp) else None
    return d
