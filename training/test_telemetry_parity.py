"""Checks that training/telemetry.py and firmware proto.c compute the same features.
usage: python test_telemetry_parity.py [path/to/esp32_camera_scripts/main]
"""
import ctypes, os, subprocess, sys, tempfile
import numpy as np
from telemetry import features

main_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(__file__), "..", "scripts", "camera_scripts", "esp32_camera_scripts", "main")
lib = os.path.join(tempfile.mkdtemp(), "proto.so")
subprocess.run(["gcc", "-O2", "-shared", "-fPIC", "-I", main_dir, os.path.join(main_dir, "proto.c"),
                "-lm", "-o", lib], check=True)
so = ctypes.CDLL(lib)


class Tel(ctypes.Structure):
    _fields_ = [("t_ms", ctypes.c_uint32), ("alt_m", ctypes.c_float), ("hdg_deg", ctypes.c_float),
                ("vz_mps", ctypes.c_float), ("vn_mps", ctypes.c_float), ("ve_mps", ctypes.c_float),
                ("yawrate_dps", ctypes.c_float), ("wp_idx", ctypes.c_uint16), ("wp_total", ctypes.c_uint16),
                ("wp_brg_deg", ctypes.c_float), ("mode", ctypes.c_int)]


so.telemetry_features.argtypes = [ctypes.POINTER(Tel), ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
rng = np.random.default_rng(0)
worst = 0.0
for _ in range(5000):
    r = dict(alt_m=rng.uniform(-5, 80), hdg_deg=rng.uniform(0, 360), vz_mps=rng.uniform(-5, 5),
             vn_mps=rng.uniform(-12, 12), ve_mps=rng.uniform(-12, 12), yawrate_dps=rng.uniform(-150, 150),
             wp_idx=int(rng.integers(0, 20)), wp_total=int(rng.integers(0, 20)), wp_brg_deg=rng.uniform(0, 360))
    dt = float(rng.uniform(0, 0.6))
    t = Tel(0, r["alt_m"], r["hdg_deg"], r["vz_mps"], r["vn_mps"], r["ve_mps"], r["yawrate_dps"],
            r["wp_idx"], r["wp_total"], r["wp_brg_deg"], 1)
    out = (ctypes.c_float * 16)()
    so.telemetry_features(ctypes.byref(t), dt, out)
    py = features(dt_s=dt, **{k: np.float32(v) for k, v in r.items()})
    worst = max(worst, float(np.abs(np.array(out[:]) - py).max()))
print(f"max |C - Python| over 5000 random inputs: {worst:.2e}")
assert worst < 1e-4, "telemetry features differ between firmware and training"
print("OK")
