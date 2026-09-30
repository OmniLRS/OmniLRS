#!/usr/bin/env python3
"""Fit the sensor_model.py parameters from real dark frames and flat fields of a flight or engineering camera.

  sensor_calibrate.py --darks darks.csv --flats flats.csv --out sensor_mycam.json [--bit-depth 12]

darks.csv: path,exposure_s,temperature_c        (lens capped; >= 2 frames per (exposure, temperature) setting,
                                                 several exposure times at each temperature, at least 2 temperatures)
flats.csv: path,exposure_s,temperature_c        (uniform diffuse target or integrating sphere, >= 2 frames per exposure,
                                                 exposures spanning ~5 % .. 90 % of saturation)
Frames: 16-bit or 8-bit PNG/TIFF straight from the camera, no gamma, no demosaic tricks (mono or one channel).

What is fitted and how (standard EMVA-1288 style):
  black_level_dn      intercept of dark mean vs exposure time at the reference temperature
  dark current        slope of dark mean (DN) vs exposure time, per temperature; converted to e-/s with the gain;
                      temp_doubling_c from the ratio between temperatures (log2 fit)
  read_noise_e        temporal noise at the shortest dark exposure: std(frame_a - frame_b)/sqrt(2), in e-
  gain_e_per_dn       photon transfer curve on flats: temporal variance vs mean (both black-level corrected),
                      slope = 1/gain  (variance in DN^2 per DN of signal)
  full_well_e         saturation DN (99.5th percentile of the brightest unclipped flat, or 2^bits-1) times gain
  prnu                std of (flat / low-pass(flat)) - 1 on a mid-level flat, after temporal averaging
  dsnu_e              std of the temporal-mean dark frame minus its low-pass, at exposure_ref, in e-
  vignetting k1,k2    least-squares fit of low-pass(flat)/max to 1 - k1 r^2 - k2 r^4, optical centre = argmax
Everything else (exposure/temperature ranges, blur) is a scene/operations choice and is kept from the defaults;
edit the JSON afterwards.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import csv
import json
import sys
from collections import defaultdict

import cv2
import numpy as np


def load(path):
    im = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if im is None:
        raise SystemExit(f"cannot read {path}")
    if im.ndim == 3:
        im = im[..., 1]  # green channel of a colour camera; mono cameras come through as HxW
    return im.astype(np.float64)


def read_csv(p):
    rows = list(csv.DictReader(open(p)))
    return [(r["path"], float(r["exposure_s"]), float(r["temperature_c"])) for r in rows]


def lowpass(im, frac=0.05):
    k = int(max(3, round(min(im.shape) * frac))) | 1
    return cv2.GaussianBlur(im, (k, k), 0)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--darks", required=True)
    ap.add_argument("--flats", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--bit-depth", type=int, default=12)
    ap.add_argument(
        "--exposure-ref", type=float, default=None, help="exposure_ref_s for the model (default: median flat exposure)"
    )
    a = ap.parse_args(argv)
    out = {"bit_depth": a.bit_depth, "name": "fitted by sensor_calibrate.py"}

    # ---------------- darks
    darks = defaultdict(list)
    for p, t, T in read_csv(a.darks):
        darks[(t, T)].append(load(p))
    temps = sorted({T for _, T in darks})
    T_ref = min(temps, key=lambda T: abs(T - 20.0))
    per_T = {}
    for T in temps:
        ts = sorted(t for t, TT in darks if TT == T)
        means = [np.mean([d.mean() for d in darks[(t, T)]]) for t in ts]
        slope, icpt = np.polyfit(ts, means, 1) if len(ts) >= 2 else (0.0, means[0])
        per_T[T] = {"slope_dn_per_s": float(slope), "black_dn": float(icpt), "exposures": ts}
    out["black_level_dn"] = per_T[T_ref]["black_dn"]
    out["temp_ref_c"] = T_ref
    # read noise from the shortest exposure pair at T_ref
    t0 = per_T[T_ref]["exposures"][0]
    fr = darks[(t0, T_ref)]
    read_dn = float(np.std(fr[0] - fr[1]) / np.sqrt(2)) if len(fr) >= 2 else float(np.std(fr[0] - lowpass(fr[0])))
    # DSNU from the temporal mean at the exposure closest to the reference
    # ---------------- flats: photon transfer curve
    flats = defaultdict(list)
    for p, t, T in read_csv(a.flats):
        flats[t].append(load(p))
    ptc = []
    for t, fr in sorted(flats.items()):
        if len(fr) < 2:
            continue
        m = 0.5 * (fr[0] + fr[1])
        mean = float(m.mean()) - out["black_level_dn"]
        var = float(np.var(fr[0] - fr[1]) / 2)
        if mean > 0 and m.mean() < 0.9 * (2**a.bit_depth - 1):
            ptc.append((mean, var, t))
    if len(ptc) < 2:
        raise SystemExit("need flats at >= 2 exposure levels with 2 frames each for the photon transfer curve")
    means = np.array([p[0] for p in ptc])
    vars_ = np.array([p[1] for p in ptc])
    k, v0 = np.polyfit(means, vars_ - read_dn**2, 1)  # var = mean / gain + const
    gain = float(1.0 / max(k, 1e-9))
    out["gain_e_per_dn"] = gain
    out["read_noise_e"] = read_dn * gain
    sat_dn = 2**a.bit_depth - 1
    out["full_well_e"] = float((sat_dn - out["black_level_dn"]) * gain)
    # dark current in e-/s and its temperature doubling
    out["dark_current_e_per_s"] = float(per_T[T_ref]["slope_dn_per_s"] * gain)
    if len(temps) >= 2:
        xs = np.array(temps)
        ys = np.log2(np.maximum([per_T[T]["slope_dn_per_s"] for T in temps], 1e-9))
        s, _ = np.polyfit(xs, ys, 1)
        out["temp_doubling_c"] = float(1.0 / max(s, 1e-6))
    # exposure reference: the flat exposure whose mean sits nearest 50 % of range
    mid = min(ptc, key=lambda p: abs(p[0] - 0.5 * (sat_dn - out["black_level_dn"])))
    out["exposure_ref_s"] = a.exposure_ref or float(
        mid[2] * (0.5 * (sat_dn - out["black_level_dn"])) / max(mid[0], 1e-6)
    )
    # PRNU + vignetting from the mid-level flat (temporal mean)
    fm = np.mean(flats[mid[2]], axis=0) - out["black_level_dn"]
    lp = lowpass(fm)
    out["prnu"] = float(np.std(fm / np.maximum(lp, 1e-6) - 1.0))
    h, w = fm.shape
    cy, cx = np.unravel_index(np.argmax(lp), lp.shape)
    y, x = np.mgrid[0:h, 0:w]
    r2 = (((x - cx) / (w / 2)) ** 2 + ((y - cy) / (h / 2)) ** 2) / (1 + (h / w) ** 2)
    g = (lp / lp.max()).ravel()
    A = np.stack([r2.ravel(), r2.ravel() ** 2], 1)
    k1, k2 = np.linalg.lstsq(A, 1.0 - g, rcond=None)[0]
    out["vignetting"] = {"k1": float(k1), "k2": float(k2), "center": [float(cx / w), float(cy / h)]}
    # DSNU at the reference exposure (dark temporal mean minus low-pass), in electrons
    t_near = min(per_T[T_ref]["exposures"], key=lambda t: abs(t - out["exposure_ref_s"]))
    dm = np.mean(darks[(t_near, T_ref)], axis=0)
    out["dsnu_e"] = float(np.std(dm - lowpass(dm)) * gain * (out["exposure_ref_s"] / t_near))
    out["_fit"] = {
        "ptc_points": [(round(m, 1), round(v, 2), t) for m, v, t in ptc],
        "read_noise_dn": read_dn,
        "per_temperature": per_T,
    }
    json.dump(out, open(a.out, "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "_fit"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
