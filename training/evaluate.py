"""Evaluates a trained run: float and int8 (the same loop the firmware runs), calibration,
recall by target size, and the tracker table used to pick thresholds. Writes model_int8.tflite
and eval.json into the run folder.

usage: python evaluate.py --run run_visdrone_v21_s1 [--data data/visdrone]
Next: python export_c.py --run <run folder>
"""
import argparse, json, os, shutil, struct, subprocess, time
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
import numpy as np
import tensorflow as tf
import dataset, models

ap = argparse.ArgumentParser()
ap.add_argument("--run", required=True)
ap.add_argument("--data", default=None, help="default: the dataset the run was trained on")
ap.add_argument("--int8-frames", type=int, default=30000, help="most test frames run through the int8 model (slow)")
a = ap.parse_args()

OUT = a.run
tj = json.load(open(f"{OUT}/train.json"))
variant, NAMES = tj["variant"], tj["classes"]
arch = variant if variant in ("v22", "v21") else "v2"
C = models.CHANNELS[arch]
K = len(NAMES)
data = a.data or tj["data"]
meta = dataset.read_meta(data)
assert meta["classes"] == NAMES, f"dataset classes {meta['classes']} differ from the run's {NAMES}"
step, frame, P = models.build(arch, K)
step.load_weights(f"{OUT}/best.weights.h5")

va, te = dataset.load(data, "val"), dataset.load(data, "test")
if len(te["y"]) == 0 or len(va["y"]) == 0:
    raise SystemExit("the dataset has no val / test streams: evaluation needs both (more flights or images)")
for d in (va, te):
    if arch == "v2":
        d["tel"][..., 8:] = 0
    if C == 2 or d["thermal"] is None:
        d["tel"][..., 12] = 0
THERMAL = C == 3 and te["thermal"] is not None


@tf.function
def run(img, tel, h):
    return step([img, tel, h], training=False)


def softmax(z):
    z = z - z.max(-1, keepdims=True); e = np.exp(z); return e / e.sum(-1, keepdims=True)


def ece(p, y, bins=15):
    conf = p.max(-1); pred = p.argmax(-1); e = 0.0
    for lo in np.linspace(0, 1, bins + 1)[:-1]:
        m = (conf > lo) & (conf <= lo + 1 / bins)
        if m.any():
            e += m.mean() * abs((pred[m] == y[m]).mean() - conf[m].mean())
    return float(e)


def frac(x):
    return float(x.mean()) if x.size else None


res = dict(variant=variant, classes=NAMES, channels=C, thermal_data=bool(THERMAL), data=os.path.abspath(data), source=meta["source"],
           state_dim=models.H, params=int(step.count_params()), macs=models.macs(arch, K))
t0 = time.time()
# ---- temperature on val ----
lv, _, hv = models.float_streams(run, va, C)
mv = va["w"] > 0
best_T, best_nll = 1.0, 1e9
for T_ in np.arange(0.5, 3.01, 0.05):
    p = softmax(lv[mv] / T_)
    nll = -np.mean(np.log(p[np.arange(len(p)), va["y"][mv]] + 1e-9))
    if nll < best_nll:
        best_T, best_nll = float(T_), nll
res["temperature"] = round(best_T, 2)

# ---- float on test ----
lt, pt, ht = models.float_streams(run, te, C)
m = te["w"] > 0
pred, y = lt.argmax(-1), te["y"]
res["float_acc"] = frac(pred[m] == y[m])
res["float_recall"] = {NAMES[c]: frac(pred[m & (y == c)] == c) for c in range(K)}
res["float_precision"] = {NAMES[c]: frac(y[m & (pred == c)] == c) for c in range(1, K)}
res["normal_fp_rate"] = frac(pred[m & (y == 0)] != 0)
res["ece_T1"] = ece(softmax(lt[m]), y[m]); res["ece_T"] = ece(softmax(lt[m] / best_T), y[m])
if THERMAL:      # the same test without the thermal frame: what the camera alone finds
    p0 = models.float_streams(run, te, C, use_thermal=False)[0].argmax(-1)
    res["float_recall_camera_only"] = {NAMES[c]: frac(p0[m & (y == c)] == c) for c in range(K)}
    res["normal_fp_rate_camera_only"] = frac(p0[m & (y == 0)] != 0)
hasm = (te["has"] > 0) & m
res["pos_mae"] = frac(np.abs(pt[hasm] - te["pos"][hasm]))
# how small a target the model still finds: recall by target size in pixels
res["recall_by_size_px"] = {}
for c in range(1, K):
    row = {}
    for lo, hi in ((3, 6), (6, 10), (10, 16), (16, 32), (32, 97)):
        s = m & (y == c) & (te["size"] >= lo) & (te["size"] < hi)
        if s.sum() >= 20:
            row[f"{lo}-{hi}"] = dict(frames=int(s.sum()), recall=frac(pred[s] == c))
    if row:
        res["recall_by_size_px"][NAMES[c]] = row
# drift: accuracy by position in the stream (no resets)
L = y.shape[1]
q = [slice(i * L // 4, (i + 1) * L // 4) for i in range(4)]
res["acc_by_stream_quarter"] = [frac(pred[:, s][m[:, s]] == y[:, s][m[:, s]]) for s in q]
res["h_absmax"] = float(np.abs(ht).max()); res["h_sat_frac"] = float((np.abs(ht) > 0.99).mean())
print("float done", round(time.time() - t0), "s", flush=True)

# ---- int8 conversion, representative data from real h trajectories ----
rng = np.random.default_rng(0)
def rep():
    n, Lv = va["y"].shape
    for _ in range(300):
        s = rng.integers(0, n); f = rng.integers(1, Lv)
        tel = va["tel"][s, f][None].copy()
        th = None
        if C == 3:               # every fourth sample without thermal, like in training
            with_th = va["thermal"] is not None and rng.random() > 0.25
            th = va["thermal"][s, f] if with_th else np.zeros((96, 96), np.uint8)
            tel[0, 12] = tel[0, 12] if with_th else 0
        img = models.stack_image(va["frames"][s, f], va["frames"][s, f - 1], th)[None]
        h0 = hv[s, f - 1][None] if rng.random() > 0.05 else np.zeros((1, models.H), np.float32)
        yield [img, tel, h0]


@tf.function(input_signature=[tf.TensorSpec([1, 96, 96, C], tf.float32, name="image"),
                              tf.TensorSpec([1, 16], tf.float32, name="telemetry"),
                              tf.TensorSpec([1, models.H], tf.float32, name="h_in")])
def serve(image, telemetry, h_in):
    lg, ps, h2 = step([image, telemetry, h_in], training=False)
    return {"logits": lg, "pos": ps, "h_out": h2}


from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2
frozen = convert_variables_to_constants_v2(serve.get_concrete_function())
_order = [t.name.split(":")[0] for t in frozen.inputs]
def rep_ordered():
    for img, tel, h0 in rep():
        by = {"image": img, "telemetry": tel, "h_in": h0}
        yield [by[n] for n in _order]
conv = tf.lite.TFLiteConverter.from_concrete_functions([frozen])
conv.optimizations = [tf.lite.Optimize.DEFAULT]
conv.representative_dataset = rep_ordered
conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
conv.inference_input_type = tf.int8
conv.inference_output_type = tf.int8
tfl = conv.convert()
open(f"{OUT}/model_int8.tflite", "wb").write(tfl)
res["tflite_bytes"] = len(tfl)

it = tf.lite.Interpreter(model_content=tfl, experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_REF)
it.allocate_tensors()
ops = {}
for o in it._get_ops_details():
    ops[o["op_name"]] = ops.get(o["op_name"], 0) + 1
res["ops"] = ops; res["n_ops"] = sum(ops.values())
ind = {d["name"]: d for d in it.get_input_details()}
outd = it.get_output_details()
def find_in(key):
    return [d for n, d in ind.items() if key in n][0]
i_img, i_tel, i_h = find_in("image"), find_in("telemetry"), find_in("h_in")
assert K not in (2, models.H), "the firmware tells the outputs apart by size"
o_lg = [d for d in outd if d["shape"][-1] == K][0]
o_ps = [d for d in outd if d["shape"][-1] == 2][0]
o_h = [d for d in outd if d["shape"][-1] == models.H][0]
res["quant"] = dict(img=[float(i_img["quantization"][0]), int(i_img["quantization"][1])],
                    h_in=[float(i_h["quantization"][0]), int(i_h["quantization"][1])],
                    h_out=[float(o_h["quantization"][0]), int(o_h["quantization"][1])],
                    tel=[float(i_tel["quantization"][0]), int(i_tel["quantization"][1])])


def q(x, d):
    s, z = d["quantization"]; return np.clip(np.rint(x / s) + z, -128, 127).astype(np.int8)


def dq(x, d):
    s, z = d["quantization"]; return (x.astype(np.float32) - z) * s


def int8_streams(d, streams):
    """firmware-equivalent loop: XOR 0x80 image path, h requantised every frame."""
    Ld = d["y"].shape[1]
    lg = np.zeros((len(streams), Ld, K), np.float32); ps = np.zeros((len(streams), Ld, 2), np.float32)
    s_in, z_in = i_h["quantization"]; s_out, z_out = o_h["quantization"]
    for j, s in enumerate(streams):
        fr = np.asarray(d["frames"][s])
        th = [np.asarray(d["thermal"][s]) if THERMAL else np.zeros_like(fr)] if C == 3 else []
        h = np.full((1, models.H), z_in, np.int8)
        for f in range(Ld):
            img = (np.stack([fr[f], fr[max(f - 1, 0)]] + [t[f] for t in th], -1)[None] ^ 0x80).view(np.int8)
            it.set_tensor(i_img["index"], img)
            it.set_tensor(i_tel["index"], q(d["tel"][s, f][None], i_tel))
            it.set_tensor(i_h["index"], h)
            it.invoke()
            ho = it.get_tensor(o_h["index"]).astype(np.float32)
            h = np.clip(np.rint((ho - z_out) * (s_out / s_in)) + z_in, -128, 127).astype(np.int8)
            lg[j, f] = dq(it.get_tensor(o_lg["index"])[0], o_lg)
            ps[j, f] = dq(it.get_tensor(o_ps["index"])[0], o_ps)
    return lg, ps


assert abs(i_img["quantization"][0] - 1.0) < 1e-3 and i_img["quantization"][1] == -128, i_img["quantization"]
# whole sequences (all tiles of a group) up to the frame budget
groups = np.unique(te["group"])
n_groups = max(1, min(len(groups), a.int8_frames // max(1, int((te["group"] == groups[0]).sum()) * L)))
sub = np.where(np.isin(te["group"], groups[:n_groups]))[0]
li, pi = int8_streams(te, sub)
predi, ms = li.argmax(-1), m[sub]
res["int8_streams"] = len(sub)
res["int8_acc"] = frac(predi[ms] == y[sub][ms])
res["int8_float_agree"] = frac(predi == pred[sub])
print("int8 done", round(time.time() - t0), "s", flush=True)

# ---- tracker events (firmware tracker.c) on calibrated int8 probabilities, per SWEEP ----
# one sweep = all tiles of a sequence at one time step: highest probability of any tile
probs = softmax(li / best_T)
gs = te["group"][sub]
seqs = np.unique(gs)
p_sw = np.stack([probs[gs == g].max(0) for g in seqs]).astype(np.float32)          # (n, L, K)
y_sw = np.stack([[(y[sub][gs == g] == c).any(0) for c in range(K)] for g in seqs])  # (n, K, L)
tms = np.stack([np.cumsum(te["dt_ms"][sub][gs == g][0]) for g in seqs]).astype(np.uint32)
n = len(seqs)
hours = float(tms[:, -1].sum()) / 3.6e6
MIN_SWEEPS = 3                                 # shorter sightings cannot be confirmed: not counted
gt = []                                        # (sequence, first sweep, last sweep + 1, class)
for s in range(n):
    for c in range(1, K):
        on = np.flatnonzero(np.diff(np.r_[0, y_sw[s, c].astype(int), 0]))
        gt += [(s, int(b0), int(b1), c) for b0, b1 in zip(on[::2], on[1::2]) if b1 - b0 >= MIN_SWEEPS]

FW_MAIN = os.environ.get("FW_MAIN", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "camera_scripts", "esp32_camera_scripts", "main"))
HARNESS = os.path.join(OUT, "trk_harness")


def events(thr, low, n_on, m_off, cd):
    """Each situation on its own (the others set to 0): the firmware tracker keeps one
    situation ACTIVE at a time, which would hide the rest in a test scene with several."""
    spans = []                               # (sequence, START sweep, END sweep, class)
    for c in range(1, K):
        one = np.zeros_like(p_sw); one[..., c] = p_sw[..., c]
        blob = struct.pack("iii", n, L, K) + one.tobytes() + tms.tobytes()
        out = subprocess.run([os.path.abspath(HARNESS), str(thr), str(low), str(n_on), str(m_off), str(cd)],
                             input=blob, capture_output=True, check=True).stdout.decode().split("\n")
        for s, f, kind, sit in (tuple(map(int, l.split())) for l in out if l.strip()):
            if kind == 1:
                spans.append([s, f, L - 1, sit])          # open until its END line (or the last sweep)
            else:
                spans[-1][2] = f
    # caught: the tracker is ACTIVE at some point of the sighting (it may start a sweep early,
    # while the target is only partly in view)
    caught, lat, used, per = 0, [], set(), {NAMES[c]: [0, 0] for c in range(1, K)}
    for (s, b0, b1, c) in gt:
        hit = [i for i, (s2, f0, f1, c2) in enumerate(spans) if s2 == s and c2 == c and f0 < b1 and f1 >= b0]
        per[NAMES[c]][1] += 1
        if hit:
            caught += 1; used.update(hit); per[NAMES[c]][0] += 1
            lat.append(max(int(tms[s, spans[hit[0]][1]]) - int(tms[s, b0]), 0) / 1000.0)
    # false alarm: ACTIVE with no sighting of that situation near it (short sightings included)
    false = sum(1 for i, (s, f0, f1, c) in enumerate(spans) if i not in used and
                not y_sw[s, c, max(f0 - n_on - 1, 0):f1 + 1].any())
    return dict(thr=thr, n_on=n_on, events=len(gt), caught=caught / max(len(gt), 1), false_total=false,
                false_per_hour=false / hours, caught_per_class={k: f"{v[0]}/{v[1]}" for k, v in per.items()},
                latency_s_median=round(float(np.median(lat)), 2) if lat else None)


res["test_hours"] = hours
res["tracker"] = []
if shutil.which("gcc"):
    subprocess.run(["gcc", "-O2", "-I", FW_MAIN, os.path.join(os.path.dirname(os.path.abspath(__file__)), "trk_harness.c"),
                    os.path.join(FW_MAIN, "tracker.c"), "-o", HARNESS], check=True)
    # cooldown 0: seeing the same target again right away is not a miss here
    res["tracker"] = [events(t, round(t - 0.2, 2), n_on, 3, 0) for t in (0.5, 0.6, 0.7, 0.8, 0.9) for n_on in (2, 3, 4)]
else:
    print("NOTE: gcc not found, tracker table skipped (install MSYS2 or use WSL)")
json.dump(res, open(f"{OUT}/eval.json", "w"), indent=1)
np.savez_compressed(f"{OUT}/test_out.npz", li=li, lt=lt, pt=pt, pi=pi)
print(json.dumps({k: v for k, v in res.items() if k not in ("tracker", "ops", "quant")}, indent=1))
print(f"\ntracker per sweep: {len(gt)} events of at least {MIN_SWEEPS} sweeps in {hours * 60:.1f} min of test data "
      f"(goal: caught >= 0.90 with <= 1 false alarm per hour):")
for r in res["tracker"]:
    print(f"  thr {r['thr']:.1f} n_on {r['n_on']}: caught {r['caught']:.2f} {r['caught_per_class']}  "
          f"false {r['false_total']} ({r['false_per_hour']:.1f}/h)  latency {r['latency_s_median']} s")
print(f"\nPut the thr / n_on you choose into situations.json, then: python export_c.py --run {OUT}")
