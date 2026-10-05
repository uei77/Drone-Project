"""Trains the Situation Selector step model on a dataset folder made by prepare_data.py.

usage: python train.py --data data/visdrone [--variant v22] [--seed 1] [--out run_name]
  v22 : v2.1 + thermal channel (default). Works on a dataset without thermal too: the channel
        is then empty, and the model can be trained further once thermal data exists.
  v21 : v2.1 architecture (dt, ego-motion, keypoint spread), camera only
  v2s : v2 architecture + stateful stream training      v2 : v2 architecture, clips from h = 0
Stage 1 teaches the image layers on single frames, stage 2 trains the whole step model on
streams with the memory carried from window to window (like the firmware runs it).
Next: python evaluate.py --run <out folder>
"""
import argparse, json, os, time
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import numpy as np
import tensorflow as tf
import keras
import dataset, models

ap = argparse.ArgumentParser()
ap.add_argument("--data", required=True)
ap.add_argument("--variant", default="v22", choices=["v22", "v21", "v2s", "v2"])
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--out", default=None)
ap.add_argument("--epochs", type=int, default=24, help="stage 2 epochs (stops early after 6 without progress)")
ap.add_argument("--iters", type=int, default=140, help="stage 2 batches per epoch")
ap.add_argument("--s1-epochs", type=int, default=4)
ap.add_argument("--s1-frames", type=int, default=150000, help="most frames used per stage 1 epoch")
a = ap.parse_args()

variant, seed = a.variant, a.seed
arch = variant if variant in ("v22", "v21") else "v2"
stateful = variant != "v2"
C = models.CHANNELS[arch]
TH_DROP = 0.25      # share of training windows with the thermal frame taken away: the model
                    # must keep working when the thermal sensor gives nothing
meta = dataset.read_meta(a.data)
NAMES = meta["classes"]; K = len(NAMES)
OUT = a.out or f"run_{os.path.basename(os.path.normpath(a.data))}_{variant}_s{seed}"
os.makedirs(OUT, exist_ok=True)
keras.utils.set_random_seed(seed)
rng = np.random.default_rng(seed)

tr = dataset.load(a.data, "train")
va = dataset.load(a.data, "val")
N, L = tr["y"].shape
T, B = 16, 16
assert N > 0, "no training streams in " + a.data
assert L >= T, f"streams must be at least {T} frames long (prepare_data.py --len)"
if len(va["y"]) == 0:
    print("WARNING: no validation streams (too few flights/images): validating on training data")
    va = {k: (v[:64] if v is not None else None) for k, v in tr.items()}
for d in (tr, va):
    if arch == "v2":
        d["tel"][..., 8:] = 0.0          # v2 design: slots 8-15 reserved (zero)
    if C == 2 or d["thermal"] is None:
        d["tel"][..., 12] = 0.0          # no thermal frame
if C == 3 and tr["thermal"] is None:
    print("NOTE: this dataset has no thermal channel: the model learns from the camera only")

step, frame, P = models.build(arch, K)
print(variant, "classes", NAMES, "step params", step.count_params(), "MAC", models.macs(arch, K))

# events are rare: weight a class by how much rarer than "normal" it is (at most x10)
use = tr["w"] > 0
cnt = np.array([int(((tr["y"] == c) & use).sum()) for c in range(K)], np.float64)
cw = np.where(cnt > 0, np.clip(cnt[0] / np.maximum(cnt, 1), 1.0, 10.0), 1.0); cw[0] = 1.0
print("usable training frames per class", dict(zip(NAMES, cnt.astype(int).tolist())), "weights", np.round(cw, 1).tolist())
if (cnt[1:] == 0).any():
    print("WARNING: no training frames for", [n for n, c in zip(NAMES[1:], cnt[1:]) if c == 0],
          "- the model cannot learn these situations from this dataset")
CW = tf.constant(cw, tf.float32)
ce = keras.losses.SparseCategoricalCrossentropy(from_logits=True, reduction="none")

# ---------------- stage 1: single frame ----------------
X1 = tr["frames"].reshape(-1, 96, 96)
T1 = tr["tel"].reshape(-1, 16)
Y1 = tr["y"].reshape(-1).copy(); POS1 = tr["pos"].reshape(-1, 2); HAS1 = tr["has"].reshape(-1)
for c in meta.get("stage1_merge", [])[1:]:      # classes that look the same in one frame -> one label
    Y1[Y1 == c] = meta["stage1_merge"][0]
ok1 = np.where(use.reshape(-1))[0]
opt1 = keras.optimizers.AdamW(3e-3, weight_decay=1e-4)


@tf.function
def s1_step(x, t, y, p, has):
    with tf.GradientTape() as tape:
        lg, ps = frame([x, t], training=True)
        loss = tf.reduce_mean(ce(y, lg) * tf.gather(CW, y)) + 2 * tf.reduce_mean(tf.reduce_sum(tf.abs(ps - p), -1) * has)
    g = tape.gradient(loss, frame.trainable_variables)
    opt1.apply_gradients(zip(g, frame.trainable_variables))
    return loss


t0 = time.time()
for ep in range(a.s1_epochs):
    idx = rng.permutation(ok1)[:a.s1_frames]
    ls = []
    for b in range(0, len(idx) - 127, 128):
        j = np.sort(idx[b:b + 128])
        ls.append(float(s1_step(X1[j][..., None].astype(np.float32), T1[j], Y1[j], POS1[j], HAS1[j])))
    print(f"s1 ep{ep} loss {np.mean(ls) if ls else float('nan'):.3f} {time.time()-t0:.0f}s", flush=True)

models.transfer_stage1(P)
models.freeze_bn(step)

# ---------------- stage 2: sequence, unrolled step model ----------------
opt2 = keras.optimizers.AdamW(1e-3, weight_decay=1e-4)


@tf.function
def s2_step(img, tel, y, pos, has, w, h0):
    with tf.GradientTape() as tape:
        h = h0
        loss = 0.0
        for t in range(T):
            lg, ps, h = step([img[:, t], tel[:, t], h], training=False)
            l = ce(y[:, t], lg) * tf.gather(CW, y[:, t]) + 2 * tf.reduce_sum(tf.abs(ps - pos[:, t]), -1) * has[:, t]
            loss += tf.reduce_sum(l * w[:, t])
        loss = loss / (tf.reduce_sum(w) + 1.0)
    g = tape.gradient(loss, step.trainable_variables)
    g, _ = tf.clip_by_global_norm(g, 5.0)
    opt2.apply_gradients(zip(g, step.trainable_variables))
    return loss, h


def batch_imgs(si, fi, tel):
    """Model images (B,T,96,96,C) for streams si at frames fi. With a thermal channel, some
    windows lose it (zeros, telemetry slot 12 = 0); tel is changed in place to match."""
    cur = np.stack([tr["frames"][s, f] for s, f in zip(si, fi)])       # B,T,96,96
    prev = np.stack([tr["frames"][s, np.maximum(f - 1, 0)] for s, f in zip(si, fi)])
    th = None
    if C == 3:
        th = np.zeros_like(cur)
        if tr["thermal"] is not None:
            keep = rng.random(len(si)) >= TH_DROP
            for b in np.where(keep)[0]:
                th[b] = tr["thermal"][si[b], fi[b]]
            tel[~keep, :, 12] = 0.0
    return models.stack_image(cur, prev, th)


# streams with a target are picked 4x as often as empty ones
p_stream = 1.0 + 3.0 * ((tr["y"] > 0) & use).any(1)
p_stream = p_stream / p_stream.sum()
pick = lambda n: rng.choice(N, n, p=p_stream)
cur_s = pick(B); cur_f = np.zeros(B, int); h_carry = np.zeros((B, models.H), np.float32)


@tf.function
def run(img, tel, h):
    return step([img, tel, h], training=False)


def validate():
    """Continuous run of every validation stream. Returns (mean recall over the classes that
    occur, share of normal frames called a situation, plain accuracy)."""
    lg, _, _ = models.float_streams(run, va, C)
    m = va["w"] > 0
    pred, y = lg.argmax(-1)[m], va["y"][m]
    rec = [float((pred[y == c] == c).mean()) for c in range(K) if (y == c).any()]
    fp = float((pred[y == 0] != 0).mean()) if (y == 0).any() else 0.0
    return float(np.mean(rec)), fp, float((pred == y).mean())


best, best_ep, patience = -1, -1, 0
hist = []
for ep in range(a.epochs):
    ls = []
    for it in range(a.iters):
        if not stateful:
            si = pick(B); f0 = rng.integers(0, L - T + 1, B)
            fi = f0[:, None] + np.arange(T)
            tel = tr["tel"][si[:, None], fi]
            img = batch_imgs(si, fi, tel)
            img[:, 0, ..., 1] = img[:, 0, ..., 0]   # no previous frame at clip start
            w = tr["w"][si[:, None], fi].copy(); w[:, :3] = 0
            h0 = np.zeros((B, models.H), np.float32)
        else:
            new_stream = cur_f + T > L
            reset = new_stream | (rng.random(B) < 0.05)
            cur_s = np.where(new_stream, pick(B), cur_s)
            cur_f = np.where(new_stream, 0, cur_f)
            h_carry[reset] = 0
            si = cur_s; fi = cur_f[:, None] + np.arange(T)
            tel = tr["tel"][si[:, None], fi]
            img = batch_imgs(si, fi, tel)
            img[cur_f == 0, 0, ..., 1] = img[cur_f == 0, 0, ..., 0]
            w = tr["w"][si[:, None], fi].copy(); w[reset, :3] = 0
            h0 = h_carry
        loss, hT = s2_step(img, tel, tr["y"][si[:, None], fi],
                           tr["pos"][si[:, None], fi], tr["has"][si[:, None], fi], w, h0)
        ls.append(float(loss))
        if stateful:
            h_carry = hT.numpy(); cur_f = cur_f + T
    rec, fp, acc = validate()
    crit = rec - fp          # find the targets without calling empty ground a target
    hist.append(dict(ep=ep, loss=float(np.mean(ls)), recall=rec, normal_fp=fp, acc=acc))
    print(f"s2 ep{ep} loss {np.mean(ls):.3f} val recall {rec:.3f} normal-fp {fp:.3f} acc {acc:.3f} {time.time()-t0:.0f}s", flush=True)
    if crit > best:
        best, best_ep, patience = crit, ep, 0
        step.save_weights(f"{OUT}/best.weights.h5")
    else:
        patience += 1
        if patience >= 6:
            break
json.dump(dict(hist=hist, best=best, best_ep=best_ep, params=step.count_params(), macs=models.macs(arch, K),
               variant=variant, classes=NAMES, data=os.path.abspath(a.data), seed=seed),
          open(f"{OUT}/train.json", "w"), indent=1)
print(f"done: best epoch {best_ep}. Next: python evaluate.py --run {OUT}")
