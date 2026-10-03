"""Laptop side of the data-collection build (firmware COLLECT_MODE 1).
Receives frames + raw telemetry over UDP, shows them live, writes one .npz per flight.

usage: python recorder.py --out flights/ [--port 5005] [--no-view]
       python recorder.py --recover flights/      finish recordings left by a crash

Live window (needs: pip install opencv-python):
  the 320x240 frame with the twelve 96x96 tiles the model will look at, fps,
  lost frames, telemetry and its age, brightness.
  keys 1-9 : rough mark "situation N is in view now" (per situations.json)
  0        : rough mark "situation ended"
  space    : rough mark "look here later"
  q / Esc  : stop and save
Marks go to flight_..._marks.json; label_tool.py shows them on the timeline.
Without a window (--no-view) stop with Ctrl+C.

Every frame goes to disk as it arrives (flight_....frames.tmp / .meta.tmp / .info.tmp), so a
crash or a flat laptop battery loses nothing: --recover turns what is there into the .npz.
If the ESP32 restarts in the air (frame numbers start again), a new flight file is started.
A frame with a few lost packets is kept: the lost parts are black and listed in `missing`,
and training skips only the tiles they touch.

Output arrays: frames (N,240,320) uint8, esp_ms (N,), tel_age_ms (N,), tel (N,11) raw fields
(see telemetry.RAW_FIELDS), frame_id (N,), missing (N,n_chunks) bool.
"""
import argparse, glob, json, os, socket, struct, threading, time
import numpy as np
from telemetry import RAW_FIELDS

import tiles

MAGIC, SLICE = 0x32564C44, 1200                                        # "DLV2", see collect.h
HDR = struct.Struct("<IIBBHHHII" + "I" + "6f" + "HH" + "f" + "i")      # packet header + telemetry_t
MODES = {0: "other", 1: "AUTO", 2: "PAUSED", 3: "RTL/LAND"}
MAX_MISSING = 8          # packets (of 64) a frame may lose and still be kept


def meta_dtype(n_chunks):
    return np.dtype([("frame_id", "<u4"), ("esp_ms", "<u4"), ("age", "<u4"),
                     ("tel", "<f8", (len(RAW_FIELDS),)), ("missing", "?", (n_chunks,))])


class Writer:
    """One flight on disk, appended frame by frame."""

    def __init__(self, out_dir, shape, n_chunks):
        base = os.path.join(out_dir, time.strftime("flight_%Y%m%d_%H%M%S"))
        k = 0
        while glob.glob(base + (f"_{k}" if k else "") + ".*"):   # same second: restart of the ESP32
            k += 1
        self.base = base + (f"_{k}" if k else "")
        self.dtype = meta_dtype(n_chunks)
        with open(self.base + ".info.tmp", "w") as f:
            json.dump(dict(shape=list(shape), n_chunks=n_chunks), f)
        self.frames = open(self.base + ".frames.tmp", "wb")
        self.meta = open(self.base + ".meta.tmp", "wb")
        self.n, self.marks = 0, []

    def add(self, rec):
        self.frames.write(rec["frame"].tobytes())
        self.meta.write(np.array([(rec["frame_id"], rec["esp_ms"], rec["age"], rec["tel"], rec["missing"])],
                                 self.dtype).tobytes())
        self.n += 1
        if self.n % 10 == 0:
            self.frames.flush(); self.meta.flush()

    def close(self):
        self.frames.close(); self.meta.close()
        return finalize(self.base, self.marks)


def finalize(base, marks=()):
    """flight_x.*.tmp -> flight_x.npz (+ _marks.json). Returns the .npz path, None if empty."""
    with open(base + ".info.tmp") as f:
        info = json.load(f)
    h, w = info["shape"]
    meta = np.fromfile(base + ".meta.tmp", meta_dtype(info["n_chunks"]))
    n = min(len(meta), os.path.getsize(base + ".frames.tmp") // (h * w))   # a crash may cut the last frame
    name = None
    if n:
        meta = meta[:n]
        frames = np.memmap(base + ".frames.tmp", np.uint8, "r", shape=(n, h, w))
        name = base + ".npz"
        np.savez_compressed(name, frames=frames, frame_id=meta["frame_id"], esp_ms=meta["esp_ms"],
                            tel_age_ms=meta["age"], tel=meta["tel"], missing=meta["missing"],
                            tel_fields=np.array(RAW_FIELDS))
        frames._mmap.close()                 # Windows cannot delete a file that is still mapped
        if marks:
            with open(base + "_marks.json", "w", encoding="utf-8") as f:
                json.dump(dict(flight=os.path.basename(name), marks=list(marks)), f, indent=1)
        print("wrote", name, n, "frames,", len(marks), "marks,",
              int(meta["missing"].any(1).sum()), "with lost packets")
    else:
        print("no frames received, nothing saved")
    for ext in (".frames.tmp", ".meta.tmp", ".info.tmp"):
        os.remove(base + ext)
    return name


def recover(folder):
    bases = [p[:-len(".info.tmp")] for p in sorted(glob.glob(os.path.join(folder, "flight_*.info.tmp")))]
    for b in bases:
        finalize(b)
    return len(bases)


class Receiver(threading.Thread):
    """Puts packets together into frames and hands them to the Writer in frame order.
    Runs apart from the window so drawing never drops packets."""

    def __init__(self, port, out_dir):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.2)
        self.out_dir = out_dir
        self.lock = threading.Lock()
        self.pending = {}                 # frame_id -> frame being received
        self.writer, self.files = None, []
        self.count, self.last = 0, None
        self.last_fid, self.closed = -1, {}   # newest closed frame; frame_id -> esp_ms of recent ones
        self.lost = self.partial = 0
        self.stop = False

    def _close(self, fid):
        """Frame fid gets no more packets: keep it (maybe with holes) or count it as lost."""
        p = self.pending.pop(fid)
        self.last_fid = max(self.last_fid, fid)
        self.closed[fid] = p["esp_ms"]
        self.closed.pop(fid - 64, None)
        missing = np.ones(p["n"], bool)
        missing[list(p["got"])] = False
        if missing.sum() > MAX_MISSING:
            self.lost += 1
            return
        self.partial += bool(missing.any())
        rec = dict(frame_id=fid, esp_ms=p["esp_ms"], age=p["age"], tel=p["tel"], missing=missing,
                   frame=np.frombuffer(bytes(p["buf"]), np.uint8).reshape(p["shape"]))
        with self.lock:
            if self.writer is None:
                self.writer = Writer(self.out_dir, p["shape"], p["n"])
            self.writer.add(rec)
            self.count += 1
            self.last = rec

    def rotate(self):
        """Ends the current flight file (ESP32 restart or end of recording)."""
        for fid in sorted(self.pending):
            self._close(fid)
        with self.lock:
            if self.writer is not None:
                name = self.writer.close()
                if name:
                    self.files.append(name)
            self.writer, self.last_fid, self.closed = None, -1, {}

    def feed(self, data):
        """One UDP packet. Returns True when it completed a frame."""
        if len(data) != HDR.size + SLICE:
            return False
        f = HDR.unpack_from(data)
        magic, fid, chunk, n, ln, w, h, esp_ms, age = f[:9]
        if magic != MAGIC or ln != SLICE or n * SLICE != w * h or chunk >= n:
            return False
        if fid <= self.last_fid:
            if self.closed.get(fid) == esp_ms:
                return False                           # late packet of a frame already closed
            print(f"frame {self.last_fid} -> {fid}: the ESP32 restarted, starting a new flight file")
            self.rotate()                              # its frame numbers and its clock start again
        p = self.pending.setdefault(fid, dict(buf=bytearray(w * h), got=set(), esp_ms=esp_ms,
                                              age=age, tel=f[9:], shape=(h, w), n=n))
        p["buf"][chunk * SLICE:(chunk + 1) * SLICE] = data[HDR.size:]
        p["got"].add(chunk)
        complete = len(p["got"]) == p["n"]
        # frames are sent one after the other: anything older than this one is finished
        for old in sorted(k for k in self.pending if k < fid or (k == fid and complete)):
            self._close(old)
        return complete

    def run(self):
        while not self.stop:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            self.feed(data)

    def snapshot(self):
        with self.lock:
            return self.count, self.last

    def mark(self, key):
        with self.lock:
            if self.writer is not None and self.last is not None:
                self.writer.marks.append(dict(t_ms=int(self.last["esp_ms"]), key=key))

    def marks(self):
        with self.lock:
            return list(self.writer.marks) if self.writer is not None else []


def render(rec, n, fps, lost, elapsed, marks, scale=2):
    """Builds the live window image (pure function: testable without a display)."""
    import cv2
    fh, fw = rec["frame"].shape if rec is not None else (tiles.CAM_H, tiles.CAM_W)
    S, SH = fw * scale, max(fh * scale, 400)
    img = np.full((SH, S + 260, 3), 32, np.uint8)
    if rec is not None:
        img[:fh * scale, :S] = cv2.cvtColor(
            cv2.resize(rec["frame"], (S, fh * scale), interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        if (fh, fw) == (tiles.CAM_H, tiles.CAM_W):          # the tiles the model looks at
            for x0, y0 in tiles.ORIGINS:
                cv2.rectangle(img, (x0 * scale, y0 * scale),
                              ((x0 + tiles.TILE) * scale - 1, (y0 + tiles.TILE) * scale - 1), (90, 140, 90), 1)
        cv2.drawMarker(img, (S // 2, fh * scale // 2), (0, 200, 255), cv2.MARKER_CROSS, 16, 1)
    S_ = SH
    def put(y, text, col=(230, 230, 230)):
        cv2.putText(img, text, (S + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    put(24, f"REC {int(elapsed) // 60:02d}:{int(elapsed) % 60:02d}   {n} frames", (80, 80, 255))
    put(48, f"{fps:4.1f} fps   lost {lost}", (0, 200, 255) if lost else (230, 230, 230))
    if rec is None:
        put(84, "waiting for frames...", (0, 200, 255))
        return img
    t = rec["tel"]; age = rec["age"]
    no_tel = age == 0xFFFFFFFF
    stale = no_tel or age > 500
    put(84, "NO telemetry" if no_tel else f"telemetry age {age} ms", (60, 60, 255) if stale else (120, 220, 120))
    if not no_tel:
        put(108, f"alt {t[1]:5.1f} m   hdg {t[2]:3.0f}")
        put(132, f"speed {np.hypot(t[4], t[5]):4.1f} m/s  vert {t[3]:+.1f}")
        put(156, f"mode {MODES.get(int(t[10]), '?')}   wp {int(t[7])}/{int(t[8])}")
    mean = float(rec["frame"].mean())
    bad = mean < 40 or mean > 215
    put(192, f"brightness {mean:5.1f}" + ("  !" if bad else ""), (60, 60, 255) if bad else (230, 230, 230))
    hist = np.bincount(rec["frame"].ravel() >> 3, minlength=32).astype(np.float32)
    hist = hist / max(hist.max(), 1)
    for i, h in enumerate(hist):
        cv2.rectangle(img, (S + 12 + i * 7, 250), (S + 17 + i * 7, 250 - int(h * 40)), (160, 160, 160), -1)
    put(284, f"marks: {len(marks)}" + (f"  last: {marks[-1]['key']}" if marks else ""))
    put(S_ - 40, "1-9 situation 0 ended space note", (150, 150, 150))
    put(S_ - 16, "q / Esc: stop and save", (150, 150, 150))
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="flights")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--no-view", action="store_true")
    ap.add_argument("--recover", metavar="FOLDER", help="finish recordings left by a crash, then exit")
    a = ap.parse_args()
    if a.recover:
        print(recover(a.recover), "unfinished recording(s) found")
        return
    os.makedirs(a.out, exist_ok=True)
    rx = Receiver(a.port, a.out)
    rx.start()
    print(f"listening on udp/{a.port}")
    t0 = time.time()
    hist = []                       # (time, frame count) for fps over the last 2 s
    try:
        if a.no_view:
            while True:
                time.sleep(5)
                n, _ = rx.snapshot()
                print(f"{n} frames, {n / (time.time() - t0):.1f} fps, lost {rx.lost}, with holes {rx.partial}")
        import cv2
        while True:
            n, rec = rx.snapshot()
            now = time.time()
            hist = [h for h in hist if now - h[0] < 2.0] + [(now, n)]
            fps = (hist[-1][1] - hist[0][1]) / max(hist[-1][0] - hist[0][0], 1e-3)
            cv2.imshow("DroneLaya recorder", render(rec, n, fps, rx.lost, now - t0, rx.marks()))
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if ord("0") <= k <= ord("9") or k == ord(" "):
                rx.mark(chr(k) if k != ord(" ") else "note")
    except KeyboardInterrupt:
        pass
    rx.stop = True
    rx.join(timeout=1)
    rx.rotate()


if __name__ == "__main__":
    main()
