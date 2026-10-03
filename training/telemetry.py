"""Telemetry -> 16 model features. Must match firmware main/proto.c telemetry_features() exactly.
test_telemetry_parity.py compiles proto.c and checks both on random inputs.
"""
import numpy as np

ALT_SCALE_M, VZ_SCALE, GS_SCALE, DT_SCALE_S, EGO_SCALE, YAWRATE_SCALE = 50.0, 3.0, 10.0, 0.30, 0.50, 90.0
RAW_FIELDS = ["t_ms", "alt_m", "hdg_deg", "vz_mps", "vn_mps", "ve_mps", "yawrate_dps",
              "wp_idx", "wp_total", "wp_brg_deg", "mode"]


def features(alt_m, hdg_deg, vz_mps, vn_mps, ve_mps, yawrate_dps, wp_idx, wp_total, wp_brg_deg, dt_s):
    """All arguments may be numpy arrays of the same shape. Returns (..., 16) float32."""
    f32 = np.float32
    h = np.deg2rad(np.asarray(hdg_deg, f32))
    ch, sh = np.cos(h), np.sin(h)
    alt_m = np.asarray(alt_m, f32)
    alt = np.maximum(alt_m, 0.5)
    vn, ve = np.asarray(vn_mps, f32), np.asarray(ve_mps, f32)
    gs = np.sqrt(vn * vn + ve * ve)
    v_fwd = vn * ch + ve * sh
    v_rgt = -vn * sh + ve * ch
    rel = np.deg2rad(np.asarray(wp_brg_deg, f32) - np.asarray(hdg_deg, f32))
    wp_total = np.asarray(wp_total, f32)
    prog = np.where(wp_total > 0, np.asarray(wp_idx, f32) / np.maximum(wp_total, 1), 0)
    out = np.zeros(np.broadcast(alt_m, h).shape + (16,), f32)
    out[..., 0] = np.clip(alt_m / ALT_SCALE_M, 0, 1)
    out[..., 1] = sh
    out[..., 2] = ch
    out[..., 3] = np.clip(np.asarray(vz_mps, f32) / VZ_SCALE, -1, 1)
    out[..., 4] = np.clip(gs / GS_SCALE, 0, 1)
    out[..., 5] = np.clip(prog, 0, 1)
    out[..., 6] = np.sin(rel)
    out[..., 7] = np.cos(rel)
    out[..., 8] = np.clip(np.asarray(dt_s, f32) / DT_SCALE_S, 0, 1)
    out[..., 9] = np.clip(v_fwd / alt / EGO_SCALE, -1, 1)
    out[..., 10] = np.clip(v_rgt / alt / EGO_SCALE, -1, 1)
    out[..., 11] = np.clip(np.asarray(yawrate_dps, f32) / YAWRATE_SCALE, -1, 1)
    return out
