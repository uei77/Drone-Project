"""Synthetic drone-camera streams for DroneLaya v2 / v2.1 experiments.

Harder than the earlier verification set on purpose:
  * ego-motion: the ground texture slides under the camera (drone is flying),
  * ground "distractor" blobs that look like objects in a single frame,
  * variable frame interval dt (frame drops, 6-9 fps with jitter),
  * slow auto-exposure drift + sensor noise.

Classes (per frame):
  0 normal      - only ground
  1 approaching - object growing
  2 receding    - object shrinking
  3 static      - object hovering, constant size (moves differently from ground)
  4 marker      - ground-fixed landing marker ("H" in a square)
Classes 1-3 have the same final-size distribution -> not separable from one frame.
"""
import numpy as np

IMG = 96
TEX = 512
K = 5  # classes incl. normal
NAMES = ["normal", "approach", "recede", "static", "marker"]


def _smooth_noise(rng, n, sigma):
    f = np.fft.fft2(rng.standard_normal((n, n)))
    ky = np.fft.fftfreq(n)[:, None]
    kx = np.fft.fftfreq(n)[None, :]
    g = np.exp(-2 * (np.pi * sigma) ** 2 * (kx ** 2 + ky ** 2))
    out = np.real(np.fft.ifft2(f * g))
    return out / (out.std() + 1e-6)


def make_texture(rng):
    t = 110 + 28 * _smooth_noise(rng, TEX, 12) + 12 * _smooth_noise(rng, TEX, 3)
    yy, xx = np.mgrid[0:TEX, 0:TEX]
    # ground distractors: disks and squares that look like objects in one frame
    for _ in range(rng.integers(25, 45)):
        cx, cy = rng.uniform(0, TEX, 2)
        r = rng.uniform(4, 16)
        val = rng.choice([rng.uniform(20, 60), rng.uniform(190, 240)])
        dx = (xx - cx + TEX / 2) % TEX - TEX / 2
        dy = (yy - cy + TEX / 2) % TEX - TEX / 2
        if rng.random() < 0.5:
            d = np.sqrt(dx ** 2 + dy ** 2)
            m = np.clip(r + 0.5 - d, 0, 1)
            if rng.random() < 0.6:  # rimmed ground disks look exactly like objects
                ring = np.clip(r + 2.0 - d, 0, 1) - m
                t = t * (1 - 0.5 * ring) + (255 - val) * 0.5 * ring
        else:
            m = ((np.abs(dx) < r) & (np.abs(dy) < r)).astype(np.float32)
        t = t * (1 - m) + val * m
    return t.astype(np.float32)


def _disk(img, cx, cy, r, val):
    yy, xx = np.mgrid[0:IMG, 0:IMG]
    d = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    m = np.clip(r + 0.5 - d, 0, 1)
    ring = np.clip(r + 2.0 - d, 0, 1) - m  # dark/bright rim, makes objects distinct-ish
    img[:] = img * (1 - m) + val * m
    img[:] = img * (1 - 0.5 * ring) + (255 - val) * 0.5 * ring


def _marker(img, cx, cy, s):
    x0, y0 = int(round(cx - s / 2)), int(round(cy - s / 2))
    x1, y1 = x0 + int(s), y0 + int(s)
    sub = np.zeros((IMG, IMG), bool)
    sub[max(y0, 0):max(min(y1, IMG), 0), max(x0, 0):max(min(x1, IMG), 0)] = True
    img[sub] = 235
    w = max(2, int(s / 6))
    h = np.zeros_like(sub)
    ys, ye = y0 + w, y1 - w
    for xa in (x0 + w, x1 - 2 * w):
        h[max(ys, 0):max(min(ye, IMG), 0), max(xa, 0):max(min(xa + w, IMG), 0)] = True
    ym = (y0 + y1) // 2
    h[max(ym - w // 2, 0):max(min(ym - w // 2 + w, IMG), 0), max(x0 + w, 0):max(min(x1 - w, IMG), 0)] = True
    img[h & sub] = 25


def make_stream(seed, length, p_event=0.5):
    """Returns dict of frames (L,96,96) uint8, tel (L,16) float32 [v2.1 layout],
    y (L,) int, pos (L,2) float [-1,1], has_pos (L,), w (L,) loss weight."""
    rng = np.random.default_rng(seed)
    tex = make_texture(rng)
    frames = np.zeros((length, IMG, IMG), np.uint8)
    tel = np.zeros((length, 16), np.float32)
    y = np.zeros(length, np.int64)
    pos = np.zeros((length, 2), np.float32)
    has = np.zeros(length, np.float32)
    w = np.ones(length, np.float32)

    cam = rng.uniform(0, TEX, 2)
    vel = rng.uniform(-2.5, 2.5, 2)
    gain = 1.0
    alt = rng.uniform(8, 25)
    heading = rng.uniform(0, 2 * np.pi)
    t = 0
    # schedule: list of (start, end, cls)
    events = []
    i = int(rng.integers(10, 60))
    while i < length - 20:
        L = int(rng.integers(16, 40))
        cls = int(rng.integers(1, 5))
        events.append((i, min(i + L, length), cls))
        i += L + int(rng.integers(20, 80))
    ev_at = np.zeros(length, np.int64)
    ev_id = -np.ones(length, np.int64)
    for k, (s, e, c) in enumerate(events):
        ev_at[s:e] = c
        ev_id[s:e] = k

    # per-event parameters, sampled lazily
    params = {}
    for f in range(length):
        dt = int(rng.choice([1, 2, 3], p=[0.5, 0.3, 0.2]))
        if rng.random() < 0.01:
            vel = rng.uniform(-2.5, 2.5, 2)
        cam = cam + vel * dt + rng.normal(0, 0.6, 2)  # vibration jitter
        gain = float(np.clip(gain + rng.normal(0, 0.01), 0.7, 1.3))
        t += dt
        ix, iy = int(round(cam[0])) % TEX, int(round(cam[1])) % TEX
        rows = (np.arange(IMG) + iy) % TEX
        cols = (np.arange(IMG) + ix) % TEX
        img = tex[np.ix_(rows, cols)].copy()

        c = int(ev_at[f])
        k = int(ev_id[f])
        if c:
            s, e, _ = events[k]
            if k not in params:
                steps_total = 0  # unknown dt ahead -> use expected 1.7/frame
                n = e - s
                ratio = rng.uniform(1.8, 3.2)
                r_end = rng.uniform(6, 15)
                pr = dict(
                    p=np.array(rng.uniform(25, 71, 2)),
                    v=rng.uniform(-0.4, 0.4, 2),
                    val=float(rng.choice([rng.uniform(15, 50), rng.uniform(200, 245)])),
                    krate=np.log(ratio) / (1.7 * n),
                    r=r_end * (ratio if c == 1 else 1 / ratio if c == 2 else 1.0) ** -1,
                    s=rng.uniform(16, 26),
                )
                if c == 4:  # ground fixed: start so it stays in view
                    pr["p"] = np.array([48.0, 48.0]) - vel * 1.7 * n / 2 + rng.uniform(-10, 10, 2)
                    pr["p"] = np.clip(pr["p"], 14, 82)
                params[k] = pr
            pr = params[k]
            if c in (1, 2, 3):
                if c == 1:
                    pr["r"] *= np.exp(pr["krate"] * dt)
                elif c == 2:
                    pr["r"] *= np.exp(-pr["krate"] * dt)
                else:
                    pr["r"] *= np.exp(rng.normal(0, 0.01))
                pr["r"] = float(np.clip(pr["r"], 2.5, 30))
                pr["p"] = pr["p"] + pr["v"] * dt
                for a in range(2):
                    if pr["p"][a] < 18 or pr["p"][a] > 78:
                        pr["v"][a] *= -1
                _disk(img, pr["p"][0], pr["p"][1], pr["r"], pr["val"])
            else:
                pr["p"] = pr["p"] - vel * dt  # moves with ground
                _marker(img, pr["p"][0], pr["p"][1], pr["s"])
            inview = (4 < pr["p"][0] < 92) and (4 < pr["p"][1] < 92)
            if inview:
                y[f] = c
                pos[f] = (pr["p"] - 48) / 48
                has[f] = 1.0
            else:
                y[f] = 0
            # ambiguity mask: motion classes need a few frames
            if f - s < (3 if c in (1, 2, 3) else 1):
                w[f] = 0.0
        else:
            # just after an event ended: memory transition
            if f > 0 and ev_at[f - 1] != 0:
                w[f] = 0.0
            if f > 1 and ev_at[f - 2] != 0:
                w[f] = 0.0
        img = img * gain + rng.normal(0, 4, img.shape)
        frames[f] = np.clip(img, 0, 255).astype(np.uint8)

        sp = float(np.linalg.norm(vel)) * 0.1 * 10 / 1.0  # px/step -> pseudo m/s
        alt += rng.normal(0, 0.05)
        tel[f, 0] = alt / 30
        tel[f, 1], tel[f, 2] = np.sin(heading), np.cos(heading)
        tel[f, 3] = rng.normal(0, 0.05)
        tel[f, 4] = sp / 10
        tel[f, 5] = f / length
        tel[f, 6], tel[f, 7] = np.sin(heading + 0.3), np.cos(heading + 0.3)
        # v2.1 additions (v2 keeps these reserved = 0)
        tel[f, 8] = dt / 3.0                     # frame interval, normalized
        tel[f, 9], tel[f, 10] = vel / 3.0        # ego-motion in image plane (from FC velocity, alt)
        tel[f, 11] = rng.normal(0, 0.05)         # yaw rate (unused by synth world, noise)
    return dict(frames=frames, tel=np.clip(tel, -1, 1), y=y, pos=pos, has=has, w=w)


def make_set(seed0, n, length):
    ss = [make_stream(seed0 + i, length) for i in range(n)]
    return {k: np.stack([s[k] for s in ss]) for k in ss[0]}


if __name__ == "__main__":
    import time
    t0 = time.time()
    s = make_stream(1, 300)
    print("300 frames in %.1fs" % (time.time() - t0))
    print("class counts", np.bincount(s["y"], minlength=K))
    from PIL import Image
    idx = [i for i in range(0, 300, 12)][:24]
    sheet = np.concatenate([np.concatenate([s["frames"][j] for j in idx[r * 6:(r + 1) * 6]], 1) for r in range(4)], 0)
    Image.fromarray(sheet).resize((sheet.shape[1] * 2, sheet.shape[0] * 2), Image.NEAREST).save("sheet.png")
    print([NAMES[s["y"][j]] for j in idx])
