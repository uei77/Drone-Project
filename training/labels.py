"""Shared label model for real flights: situations.json, the per-flight label file,
and the conversion from events (in time) to per-frame training targets.

Used by label_tool.py (editing) and real_data.py (training). No GUI code here.
Label files store TIME (esp_ms), never frame numbers, so they stay valid when training
picks frames at a different rate than they were recorded.
"""
import json, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CLICK_HOLD_MS = 300       # a clicked target position is trusted this long before / after the click


def load_situations(path=None):
    """Returns (version, classes). classes[0] must be 'normal'."""
    with open(path or os.path.join(HERE, "situations.json"), encoding="utf-8") as f:
        s = json.load(f)
    cls = s["classes"]
    assert cls[0]["name"] == "normal", "first class must be 'normal'"
    assert len({c["name"] for c in cls}) == len(cls), "duplicate class name"
    assert len({c["key"] for c in cls}) == len(cls), "duplicate key"
    return s["version"], cls


def labels_path(npz_path):
    return os.path.splitext(npz_path)[0] + "_labels.json"


def marks_path(npz_path):
    return os.path.splitext(npz_path)[0] + "_marks.json"


class LabelDoc:
    """Events and excluded ranges of one flight."""

    def __init__(self, flight, situations_version, labeler=""):
        self.flight = os.path.basename(flight)
        self.situations_version = situations_version
        self.labeler = labeler
        self.events = []    # dict(situation, start_ms, end_ms, targets=[dict(t_ms, x, y)], note)
        self.exclude = []   # dict(start_ms, end_ms, reason)

    # ---------- file ----------
    @classmethod
    def load(cls, path, flight, situations_version):
        d = cls(flight, situations_version)
        if os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                j = json.load(f)
            d.situations_version = j.get("situations_version", situations_version)
            d.labeler = j.get("labeler", "")
            d.events = j.get("events", [])
            d.exclude = j.get("exclude", [])
            for e in d.events:
                e.setdefault("targets", []); e.setdefault("note", "")
        return d

    def save(self, path):
        self.events.sort(key=lambda e: e["start_ms"])
        self.exclude.sort(key=lambda e: e["start_ms"])
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(dict(flight=self.flight, situations_version=self.situations_version,
                           labeler=self.labeler, events=self.events, exclude=self.exclude),
                      f, indent=1, ensure_ascii=False)
        os.replace(tmp, path)   # never leaves a half-written label file

    # ---------- editing ----------
    def event_at(self, t_ms):
        for e in self.events:
            if e["start_ms"] <= t_ms <= e["end_ms"]:
                return e
        return None

    def add_event(self, situation, start_ms, end_ms):
        """Adds an event; anything it overlaps is trimmed or removed (one situation at a time)."""
        start_ms, end_ms = int(min(start_ms, end_ms)), int(max(start_ms, end_ms))
        kept = []
        for e in self.events:
            if e["end_ms"] < start_ms or e["start_ms"] > end_ms:
                kept.append(e)
            elif e["start_ms"] < start_ms:          # keep the part before
                e["end_ms"] = start_ms - 1
                e["targets"] = [t for t in e["targets"] if t["t_ms"] <= e["end_ms"]]
                kept.append(e)
            elif e["end_ms"] > end_ms:              # keep the part after
                e["start_ms"] = end_ms + 1
                e["targets"] = [t for t in e["targets"] if t["t_ms"] >= e["start_ms"]]
                kept.append(e)
        ev = dict(situation=situation, start_ms=start_ms, end_ms=end_ms, targets=[], note="")
        kept.append(ev)
        self.events = sorted(kept, key=lambda e: e["start_ms"])
        return ev

    def delete_at(self, t_ms):
        e = self.event_at(t_ms)
        if e:
            self.events.remove(e)
        return e is not None

    def set_boundary(self, t_ms, which):
        """Moves the start or end of the event at/nearest to t_ms to t_ms."""
        if not self.events:
            return False
        e = self.event_at(t_ms) or min(
            self.events, key=lambda e: min(abs(e["start_ms"] - t_ms), abs(e["end_ms"] - t_ms)))
        if which == "start" and t_ms < e["end_ms"]:
            e["start_ms"] = int(t_ms)
        elif which == "end" and t_ms > e["start_ms"]:
            e["end_ms"] = int(t_ms)
        else:
            return False
        e["targets"] = [t for t in e["targets"] if e["start_ms"] <= t["t_ms"] <= e["end_ms"]]
        return True

    def set_target(self, t_ms, x, y):
        e = self.event_at(t_ms)
        if not e:
            return False
        e["targets"] = [t for t in e["targets"] if t["t_ms"] != int(t_ms)]
        e["targets"].append(dict(t_ms=int(t_ms), x=round(float(np.clip(x, -1, 1)), 3),
                                 y=round(float(np.clip(y, -1, 1)), 3)))
        e["targets"].sort(key=lambda t: t["t_ms"])
        return True

    def clear_target(self, t_ms):
        e = self.event_at(t_ms)
        if not e:
            return False
        n = len(e["targets"])
        e["targets"] = [t for t in e["targets"] if t["t_ms"] != int(t_ms)]
        return len(e["targets"]) != n

    def add_exclude(self, start_ms, end_ms, reason=""):
        self.exclude.append(dict(start_ms=int(min(start_ms, end_ms)), end_ms=int(max(start_ms, end_ms)),
                                 reason=reason))

    def exclude_at(self, t_ms):
        for e in self.exclude:
            if e["start_ms"] <= t_ms <= e["end_ms"]:
                return e
        return None

    # ---------- targets for training ----------
    def target_at(self, e, t_ms):
        """Interpolated (x, y) of event e at t_ms. None when the position is not known there:
        the event has no clicks, or t_ms is more than CLICK_HOLD_MS before the first / after the
        last click (the target moves through the frame, so a click says nothing about later frames)."""
        ts = e["targets"]
        if not ts:
            return None
        tt = [t["t_ms"] for t in ts]
        if t_ms < tt[0] - CLICK_HOLD_MS or t_ms > tt[-1] + CLICK_HOLD_MS:
            return None
        return (float(np.interp(t_ms, tt, [t["x"] for t in ts])),
                float(np.interp(t_ms, tt, [t["y"] for t in ts])))

    def frame_labels(self, esp_ms, classes, lead_frames=1, tail_frames=2):
        """Per-frame targets for the frames at times esp_ms (already subsampled).
        Returns y (int), pos (n,2), has (n,), w (n,) like synth.make_stream.
          w = 0 : excluded ranges, the first lead_frames of each event and the tail_frames
                  after it (the memory needs time to switch)."""
        esp_ms = np.asarray(esp_ms, np.int64)
        n = len(esp_ms)
        idx = {c["name"]: i for i, c in enumerate(classes)}
        uses = {c["name"]: c.get("uses_position", False) for c in classes}
        y = np.zeros(n, np.int64); pos = np.zeros((n, 2), np.float32)
        has = np.zeros(n, np.float32); w = np.ones(n, np.float32)
        for e in self.events:
            if e["situation"] not in idx:
                raise ValueError(f"unknown situation '{e['situation']}' in {self.flight}")
            m = np.where((esp_ms >= e["start_ms"]) & (esp_ms <= e["end_ms"]))[0]
            if len(m) == 0:
                continue
            y[m] = idx[e["situation"]]
            w[m[:lead_frames]] = 0
            w[m[-1] + 1:m[-1] + 1 + tail_frames] = 0
            if uses[e["situation"]]:
                for i in m:
                    p = self.target_at(e, esp_ms[i])
                    if p:
                        pos[i] = p; has[i] = 1
        for x in self.exclude:
            w[(esp_ms >= x["start_ms"]) & (esp_ms <= x["end_ms"])] = 0
        return y, pos, has, w

    def counts(self, classes):
        """events and total seconds per situation, for the progress panel."""
        out = {c["name"]: [0, 0.0] for c in classes[1:]}
        for e in self.events:
            if e["situation"] in out:
                out[e["situation"]][0] += 1
                out[e["situation"]][1] += (e["end_ms"] - e["start_ms"]) / 1000.0
        return out
