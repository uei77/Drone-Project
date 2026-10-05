"""Situation Selector step models.
  v2  : first design                      image channels: current, previous
  v21 : + dt, ego-motion, keypoint spread image channels: current, previous
  v22 : v21 + thermal                     image channels: current, previous, thermal

Step model = one frame:  [image(96,96,C), telemetry(16), h_in(32)] -> [state logits, pos, h_out]
Thermal channel: the thermal frame brought onto the camera grid, 128 = the frame's median
temperature, 8 counts per degree; all 0 when there is no thermal frame (telemetry slot 12
says which: 1 = present).
The same step model is unrolled for training, so training and export use identical weights
(no seq->step weight copy).
"""
import numpy as np
import keras
from keras import layers

H = 32
CHANNELS = {"v2": 2, "v21": 2, "v22": 3}
GRID = np.linspace(-1, 1, 6).astype(np.float32)


def _coords(power):
    xs, ys = np.meshgrid(GRID, GRID)  # row-major 6x6 -> 36
    c = np.stack([xs.reshape(-1) ** power, ys.reshape(-1) ** power], 1)  # (36,2)
    return c


class ClipUnit(layers.Layer):
    """h' is already inside [-1, 1] (convex mix of h and tanh). The explicit clip
    (MINIMUM/MAXIMUM ops) pins the int8 range of h_out to [-1, 1] and stops the
    converter from folding the GRU output add into the next FULLY_CONNECTED bias."""
    def call(self, x):
        return keras.ops.clip(x, -1.0, 1.0)


class Parts:
    def __init__(self, variant, K):
        self.variant = variant
        L = layers
        self.conv1 = L.Conv2D(8, 3, 2, padding="same", use_bias=False, name="conv1")
        self.conv1a = L.Conv2D(8, 3, 2, padding="same", use_bias=False, name="conv1_single")
        self.bn1 = L.BatchNormalization(name="bn1")
        self.blocks = []
        for i, (c, s) in enumerate([(16, 2), (32, 1), (32, 2), (64, 1), (64, 2), (96, 1)]):
            self.blocks.append((
                L.DepthwiseConv2D(3, s, padding="same", use_bias=False, name=f"dw{i}"),
                L.BatchNormalization(name=f"dwbn{i}"),
                L.Conv2D(c, 1, use_bias=False, name=f"pw{i}"),
                L.BatchNormalization(name=f"pwbn{i}"),
            ))
        self.kp = L.Conv2D(8, 1, name="kp_conv")
        self.coord1 = L.Dense(2, use_bias=False, trainable=False, name="coord_xy")
        self.coord2 = L.Dense(2, use_bias=False, trainable=False, name="coord_xy2")
        self.tel = L.Dense(16, activation="relu", name="tel_enc")
        # GRU, one Dense per gate (maps to FULLY_CONNECTED, no SPLIT op)
        self.Wz = L.Dense(H, name="Wz"); self.Wr = L.Dense(H, name="Wr"); self.Wh = L.Dense(H, name="Wh")
        self.Uz = L.Dense(H, name="Uz"); self.Ur = L.Dense(H, name="Ur"); self.Uh = L.Dense(H, name="Uh")
        self.state_head = L.Dense(K, name="state")
        self.pos_head = L.Dense(2, activation="tanh", name="pos")
        # stage-1 head
        self.s1 = L.Dense(48, activation="relu", name="s1_dense")
        self.s1_state = L.Dense(K, name="s1_state")
        self.s1_pos = L.Dense(2, activation="tanh", name="s1_pos")

    def encode(self, img, single=False):
        L = layers
        x = L.Rescaling(1 / 255.0)(img)
        x = (self.conv1a if single else self.conv1)(x)
        x = L.ReLU(6.0)(self.bn1(x))
        for dw, dbn, pw, pbn in self.blocks:
            x = L.ReLU(6.0)(dbn(dw(x)))
            x = L.ReLU(6.0)(pbn(pw(x)))
        gap = L.GlobalAveragePooling2D()(x)
        k = self.kp(x)                       # 6x6x8
        k = L.Reshape((36, 8))(k)
        k = L.Permute((2, 1))(k)             # 8x36
        p = L.Softmax(axis=-1)(k)
        m1 = self.coord1(p)                  # 8x2 expected (x,y)
        feats = [gap, L.Flatten()(m1)]
        if self.variant != "v2":
            m2 = self.coord2(p)              # 8x2 E[x^2],E[y^2]
            var = L.Subtract()([m2, L.Multiply()([m1, m1])])
            feats.append(L.Flatten()(L.Rescaling(4.0)(var)))   # spread = size cue
        return L.Concatenate()(feats)

    def init_fixed(self):
        self.coord1.set_weights([_coords(1)])
        if self.coord2.built:
            self.coord2.set_weights([_coords(2)])


def build(variant, K):
    """K = number of classes incl. normal. Returns (step model, stage-1 frame model, parts)."""
    P = Parts(variant, K)
    L = layers
    img = keras.Input((96, 96, CHANNELS[variant]), batch_size=None, name="image")
    tel = keras.Input((16,), name="telemetry")
    h = keras.Input((H,), name="h_in")
    e = P.encode(img)
    t = P.tel(tel)
    x = L.Concatenate()([e, t])
    z = L.Activation("sigmoid")(L.Add()([P.Wz(x), P.Uz(h)]))
    r = L.Activation("sigmoid")(L.Add()([P.Wr(x), P.Ur(h)]))
    hh = L.Activation("tanh")(L.Add()([P.Wh(x), L.Multiply()([r, P.Uh(h)])]))
    h2 = L.Add()([hh, L.Multiply()([z, L.Subtract()([h, hh])])])   # z*h + (1-z)*hh
    h2 = ClipUnit(name="h_clip")(h2)
    logits = P.state_head(h2)
    pos = P.pos_head(L.Concatenate()([h2, e]))
    step = keras.Model([img, tel, h], [logits, pos, h2], name=f"selector_{variant}")

    img1 = keras.Input((96, 96, 1), name="image1")
    tel1 = keras.Input((16,), name="telemetry1")
    e1 = P.encode(img1, single=True)
    f1 = P.s1(L.Concatenate()([e1, P.tel(tel1)]))
    frame = keras.Model([img1, tel1], [P.s1_state(f1), P.s1_pos(f1)], name="stage1")
    P.init_fixed()
    return step, frame, P


def transfer_stage1(P):
    w = P.conv1a.get_weights()[0]          # 3x3x1x8: goes to the "current image" channel
    w2 = np.concatenate([w] + [np.zeros_like(w)] * (CHANNELS[P.variant] - 1), axis=2)
    P.conv1.set_weights([w2])


def freeze_bn(model):
    for l in model.layers:
        if isinstance(l, layers.BatchNormalization):
            l.trainable = False


def macs(variant, K):
    m = 48 * 48 * 8 * 9 * CHANNELS[variant]
    hw, cin = 48, 8
    for c, s in [(16, 2), (32, 1), (32, 2), (64, 1), (64, 2), (96, 1)]:
        hw //= s
        m += hw * hw * cin * 9 + hw * hw * cin * c
        cin = c
    m += 36 * 96 * 8 + 8 * 36 * 2 * (2 if variant != "v2" else 1)
    e = 128 if variant != "v2" else 112
    m += 16 * 16 + (e + 16) * H * 3 + H * H * 3 + H * K + (H + e) * 2
    return m


def stack_image(cur, prev, thermal=None):
    """(..., 96, 96) uint8 arrays -> float32 model image (..., 96, 96, C)."""
    return np.stack([cur, prev] + ([thermal] if thermal is not None else []), -1).astype(np.float32)


def float_streams(run, d, channels=2, bs=512, use_thermal=True):
    """Runs every stream of d (dataset.load) from h = 0 to its end, no resets.
    run(img, tel, h) -> (logits, pos, h) is the step model as a tf.function.
    channels = 3: the thermal channel is added (zeros and telemetry slot 12 = 0 when the dataset
    has none or use_thermal is False).
    Returns logits (N,L,K), pos (N,L,2), h (N,L,H) as numpy."""
    n, L = d["y"].shape
    out = None
    with_th = use_thermal and d.get("thermal") is not None
    for a in range(0, n, bs):
        fr = np.asarray(d["frames"][a:a + bs])
        tel = d["tel"][a:a + bs].copy()
        th = None
        if channels == 3:
            th = np.asarray(d["thermal"][a:a + bs]) if with_th else np.zeros_like(fr)
            if not with_th:
                tel[..., 12] = 0
        h = np.zeros((len(fr), H), np.float32)
        for f in range(L):
            img = stack_image(fr[:, f], fr[:, max(f - 1, 0)], None if th is None else th[:, f])
            lg, ps, h = (x.numpy() for x in run(img, tel[:, f], h))
            if out is None:
                out = (np.zeros((n, L, lg.shape[-1]), np.float32), np.zeros((n, L, 2), np.float32),
                       np.zeros((n, L, H), np.float32))
            out[0][a:a + bs, f], out[1][a:a + bs, f], out[2][a:a + bs, f] = lg, ps, h
    return out
