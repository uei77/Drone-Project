"""Event labelling tool for recorded flights.

usage: python label_tool.py flights/flight_YYYYMMDD_HHMMSS.npz [--labeler name]
needs: pip install opencv-python

Every change is saved at once to flight_..._labels.json (time based, see labels.py).
Frames outside any event and outside excluded ranges count as "normal".

Keys
  right / d , left / a   one frame forward / back      D / A   ten frames
  space                  play / pause                   n / p   next / previous event or mark
  1..9                   start an event of that situation at this frame (key from situations.json)
  Enter                  end the open event at this frame
  [  ]                   move the start / end of the event here to this frame
  x                      delete the event under the cursor
  click on the image     target centre at this frame. Needed on a few frames of EVERY event:
                         it tells training which tile the target is in
  t                      remove the target point at this frame
  k                      start / end an excluded range (take-off, landing, broken frames)
  z                      delete the excluded range under the cursor
  click on the timeline  jump there
  q / Esc                quit
"""
import argparse, json, os, sys
import numpy as np
import tiles
from labels import LabelDoc, load_situations, labels_path, marks_path

PANEL = 300
TL_H = 70                 # timeline height
COLORS = [(140, 140, 140), (80, 80, 255), (80, 220, 80), (255, 170, 60), (60, 200, 255),
          (220, 100, 220), (200, 220, 60), (120, 160, 255), (255, 120, 160), (160, 255, 200)]
TARGET_EVENTS, TARGET_FRAMES = 40, 1500


class Session:
    """All state and every edit action. No window code: the tests drive this directly."""

    def __init__(self, npz_path, situations=None, labeler=""):
        d = np.load(npz_path)
        self.frames, self.esp_ms = d["frames"], d["esp_ms"].astype(np.int64)
        self.fh, self.fw = self.frames.shape[1:]
        self.scale = 2 if self.fw >= 320 else 5
        self.sw, self.sh = self.fw * self.scale, max(self.fh * self.scale, 420)   # image area on screen
        self.tel = d["tel"] if "tel" in d else None
        self.n = len(self.frames)
        self.version, self.classes = load_situations(situations)
        self.by_key = {c["key"]: c for c in self.classes[1:]}
        self.path = labels_path(npz_path)
        self.doc = LabelDoc.load(self.path, npz_path, self.version)
        if labeler:
            self.doc.labeler = labeler
        self.marks = []
        if os.path.exists(marks_path(npz_path)):
            with open(marks_path(npz_path), encoding="utf-8") as f:
                self.marks = json.load(f).get("marks", [])
        self.i = 0
        self.open = None            # (situation, start_ms) of the event being drawn
        self.open_ex = None         # start_ms of the excluded range being drawn
        self.playing = False
        self.msg = ""
        if self.doc.situations_version != self.version:
            self.msg = f"WARNING: labels are v{self.doc.situations_version}, the list is v{self.version}"

    @property
    def t(self):
        return int(self.esp_ms[self.i])

    def save(self):
        self.doc.save(self.path)

    def step(self, k):
        self.i = int(np.clip(self.i + k, 0, self.n - 1))

    def seek_ms(self, t_ms):
        self.i = int(np.clip(np.searchsorted(self.esp_ms, t_ms), 0, self.n - 1))

    def jump(self, direction):
        pts = sorted({e["start_ms"] for e in self.doc.events} | {m["t_ms"] for m in self.marks})
        nxt = [p for p in pts if p > self.t] if direction > 0 else [p for p in pts if p < self.t]
        if nxt:
            self.seek_ms(nxt[0] if direction > 0 else nxt[-1])

    def key_class(self, ch):
        if ch not in self.by_key:
            return False
        if self.open:                                   # a new start closes the open event
            self.end_event()
        self.open = (self.by_key[ch]["name"], self.t)
        self.msg = f"{self.open[0]} started; Enter to end"
        return True

    def end_event(self):
        if not self.open:
            return False
        name, start = self.open
        self.doc.add_event(name, start, self.t)
        self.open = None
        self.msg = f"{name} saved"
        self.save()
        return True

    def delete(self):
        if self.doc.delete_at(self.t):
            self.msg = "event deleted"; self.save(); return True
        return False

    def boundary(self, which):
        if self.doc.set_boundary(self.t, which):
            self.msg = f"event {which} moved"; self.save(); return True
        return False

    def click_image(self, px, py):
        e = self.doc.event_at(self.t)
        if not e:
            self.msg = "mark the event first"; return False
        if not next(c for c in self.classes if c["name"] == e["situation"]).get("uses_position"):
            self.msg = "this situation has no position"; return False
        self.doc.set_target(self.t, px / (self.fw * self.scale / 2) - 1, py / (self.fh * self.scale / 2) - 1)
        self.msg = "target point added"; self.save()
        return True

    def clear_target(self):
        if self.doc.clear_target(self.t):
            self.msg = "target point removed"; self.save(); return True
        return False

    def toggle_exclude(self):
        if self.open_ex is None:
            self.open_ex = self.t
            self.msg = "excluded range started; k to end"
        else:
            self.doc.add_exclude(self.open_ex, self.t)
            self.open_ex = None
            self.msg = "excluded range saved"; self.save()

    def delete_exclude(self):
        x = self.doc.exclude_at(self.t)
        if x:
            self.doc.exclude.remove(x); self.msg = "excluded range deleted"; self.save()
        return x is not None

    def frame_count(self, e):
        return int(((self.esp_ms >= e["start_ms"]) & (self.esp_ms <= e["end_ms"])).sum())


def render(s):
    """Draws the whole window as an image (pure function of the session)."""
    import cv2
    S, SH, IH = s.sw, s.sh, s.fh * s.scale          # image width, area height, image height
    Wd = S + PANEL
    img = np.full((SH + TL_H, Wd, 3), 28, np.uint8)
    img[:IH, :S] = cv2.cvtColor(cv2.resize(s.frames[s.i], (S, IH), interpolation=cv2.INTER_NEAREST),
                                cv2.COLOR_GRAY2BGR)
    if (s.fh, s.fw) == (tiles.CAM_H, tiles.CAM_W):       # faint grid: the tiles the model sees
        for x0, y0 in tiles.ORIGINS:
            cv2.rectangle(img, (x0 * s.scale, y0 * s.scale),
                          ((x0 + tiles.TILE) * s.scale - 1, (y0 + tiles.TILE) * s.scale - 1), (70, 90, 70), 1)
    idx = {c["name"]: i for i, c in enumerate(s.classes)}
    e = s.doc.event_at(s.t)
    if e:
        col = COLORS[idx[e["situation"]] % len(COLORS)]
        cv2.rectangle(img, (0, 0), (S - 1, IH - 1), col, 3)
        p = s.doc.target_at(e, s.t)
        if p:
            c = (int((p[0] + 1) * S / 2), int((p[1] + 1) * IH / 2))
            exact = any(t["t_ms"] == s.t for t in e["targets"])
            cv2.circle(img, c, 9, col, 2 if exact else 1)
            cv2.drawMarker(img, c, col, cv2.MARKER_CROSS, 14, 1)

    def put(y, text, col=(230, 230, 230), sc=0.48):
        cv2.putText(img, text, (S + 12, y), cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1, cv2.LINE_AA)
    put(22, f"frame {s.i + 1}/{s.n}   {(s.t - int(s.esp_ms[0])) / 1000:7.2f} s")
    put(44, "state: " + (e["situation"] if e else ("EXCLUDED" if s.doc.exclude_at(s.t) else "normal")),
        COLORS[idx[e["situation"]] % len(COLORS)] if e else (200, 200, 200))
    if s.open:
        put(66, f"OPEN: {s.open[0]} (Enter)", (60, 200, 255))
    if s.open_ex is not None:
        put(66, "OPEN: excluded range (k)", (60, 200, 255))
    y = 100
    cnt = s.doc.counts(s.classes)
    for c in s.classes[1:]:
        n_ev, sec = cnt[c["name"]]
        fr = sum(s.frame_count(ev) for ev in s.doc.events if ev["situation"] == c["name"])
        put(y, f"[{c['key']}] {c['name']}", COLORS[idx[c["name"]] % len(COLORS)])
        put(y + 18, f"    {n_ev} events  {fr} frames  {sec:.0f} s", (190, 190, 190), 0.42)
        y += 44
    put(y + 6, f"goal: {TARGET_EVENTS} events, {TARGET_FRAMES} frames each", (130, 130, 130), 0.4)
    put(y + 24, "(sum over all flights)", (130, 130, 130), 0.4)
    put(SH - 34, "<- -> frame  space play  x delete", (150, 150, 150), 0.42)
    put(SH - 16, "[ ] edge  k exclude  n/p jump  q quit", (150, 150, 150), 0.42)
    if s.msg:
        put(SH - 58, s.msg[:38], (60, 220, 255), 0.45)

    # timeline
    t0, t1 = int(s.esp_ms[0]), int(s.esp_ms[-1])
    X = lambda t: int((t - t0) / max(t1 - t0, 1) * (Wd - 1))
    y0 = SH + 8
    cv2.rectangle(img, (0, y0), (Wd - 1, y0 + 36), (60, 60, 60), -1)
    for x in s.doc.exclude:
        cv2.rectangle(img, (X(x["start_ms"]), y0), (X(x["end_ms"]), y0 + 36), (30, 30, 30), -1)
    for ev in s.doc.events:
        cv2.rectangle(img, (X(ev["start_ms"]), y0 + 4), (max(X(ev["end_ms"]), X(ev["start_ms"]) + 2), y0 + 32),
                      COLORS[idx.get(ev["situation"], 0) % len(COLORS)], -1)
        for tg in ev["targets"]:
            cv2.line(img, (X(tg["t_ms"]), y0 + 4), (X(tg["t_ms"]), y0 + 10), (255, 255, 255), 1)
    if s.open:
        cv2.rectangle(img, (X(s.open[1]), y0 + 4), (X(s.t), y0 + 32), (60, 200, 255), 1)
    for m in s.marks:                                   # rough marks made during the flight
        cv2.line(img, (X(m["t_ms"]), y0 + 38), (X(m["t_ms"]), y0 + 50), (0, 220, 255), 2)
    cv2.line(img, (X(s.t), y0 - 4), (X(s.t), y0 + 52), (255, 255, 255), 1)
    return img


def timeline_ms(s, px):
    t0, t1 = int(s.esp_ms[0]), int(s.esp_ms[-1])
    return t0 + px / (s.sw + PANEL - 1) * (t1 - t0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("flight")
    ap.add_argument("--situations", default=None)
    ap.add_argument("--labeler", default="")
    a = ap.parse_args()
    import cv2
    s = Session(a.flight, a.situations, a.labeler)
    win = "DroneLaya labelling - " + os.path.basename(a.flight)

    def on_mouse(ev, x, y, flags, _):
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        if y < s.fh * s.scale and x < s.sw:
            s.click_image(x, y)
        elif y >= s.sh:
            s.seek_ms(timeline_ms(s, x))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    LEFT, RIGHT = (2424832, 65361), (2555904, 65363)     # Windows, Linux arrow key codes
    while True:
        cv2.imshow(win, render(s))
        k = cv2.waitKeyEx(60 if s.playing else 30)
        if s.playing:
            s.step(1)
            if s.i == s.n - 1:
                s.playing = False
        if k == -1:
            continue
        ch = chr(k & 0xFF) if 0 < (k & 0xFF) < 128 else ""
        if ch == "q" or k == 27:
            break
        elif k in RIGHT or ch == "d": s.step(1)
        elif k in LEFT or ch == "a": s.step(-1)
        elif ch == "D": s.step(10)
        elif ch == "A": s.step(-10)
        elif ch == " ": s.playing = not s.playing
        elif ch == "n": s.jump(1)
        elif ch == "p": s.jump(-1)
        elif ch in ("\r", "\n"): s.end_event()
        elif ch == "x": s.delete()
        elif ch == "[": s.boundary("start")
        elif ch == "]": s.boundary("end")
        elif ch == "t": s.clear_target()
        elif ch == "k": s.toggle_exclude()
        elif ch == "z": s.delete_exclude()
        elif ch: s.key_class(ch)
    s.save()
    cv2.destroyAllWindows()
    print("saved", s.path, {k: v[0] for k, v in s.doc.counts(s.classes).items()})


if __name__ == "__main__":
    main()
