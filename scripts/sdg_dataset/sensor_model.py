#!/usr/bin/env python3
"""Sensor realism for rendered frames, applied in post (labels untouched, re-randomised per epoch).

Pipeline (per image, all stages optional, parameters from a JSON calibration file):
  8-bit sRGB render -> linear radiance -> exposure variation (stops) -> optics: vignetting, defocus blur,
  motion blur -> photo-electrons (full well) -> PRNU -> shot noise (Poisson) -> dark current (temperature,
  exposure time) with its own shot noise -> DSNU -> read noise (Gaussian) -> gain (e-/DN) + black level ->
  quantisation (bit depth) + clipping -> back to 8-bit sRGB (same range as the clean render).

Use as a library (numpy in, numpy out; works inside albumentations via A.Lambda):
    from sensor_model import SensorModel
    sm = SensorModel.from_json("sensor_default.json")
    noisy = sm(img_rgb_uint8, seed=hash((frame_id, epoch)) & 0xFFFFFFFF)
or as a CLI to materialise a set:
    sensor_model.py --calib sensor_default.json --in images/ --out images_sensor/
Determinism: the seed fixes every random draw (exposure, temperature, blur, noise), so a training run can log
(frame_id, epoch) -> seed and reproduce any sample. Calibrate with sensor_calibrate.py from dark frames and
flat fields of the flight or engineering camera; the defaults below are generic 12-bit CMOS numbers.
"""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import argparse
import json
import sys
import zlib
from pathlib import Path

import cv2
import numpy as np

DEFAULT = {
    "name": "generic-12bit-cmos (placeholder: calibrate with sensor_calibrate.py)",
    "bit_depth": 12,
    "black_level_dn": 64,
    "full_well_e": 10000.0,  # electrons at saturation (white in the render maps to this at nominal exposure)
    "gain_e_per_dn": 2.5,  # conversion gain; flat-field PTC slope
    "read_noise_e": 3.5,  # rms electrons; from dark frames at the shortest exposure
    "dark_current_e_per_s": 8.0,  # at temp_ref_c
    "temp_ref_c": 20.0,
    "temp_doubling_c": 6.5,  # dark current doubles every N degrees C
    "prnu": 0.01,  # rms pixel gain non-uniformity (fraction)
    "dsnu_e": 2.0,  # rms fixed-pattern dark offset (electrons at exposure_ref)
    "exposure_ref_s": 0.01,  # exposure that maps render white to full well
    "exposure_s_range": [0.002, 0.04],  # per-frame exposure time draw (log-uniform)
    "exposure_ev_sigma": 0.5,  # extra gain jitter in stops (auto-exposure error)
    "temperature_c_range": [-20.0, 40.0],
    "vignetting": {"k1": 0.35, "k2": 0.15, "center": [0.5, 0.5]},  # gain = 1 - k1 r^2 - k2 r^4, r = normalised radius
    "defocus_sigma_px_range": [0.0, 1.2],
    "motion_blur_px_range": [0.0, 4.0],
    "motion_blur_prob": 0.3,
    "fixed_pattern_seed": 12345,  # PRNU/DSNU maps are a property of the sensor: fixed across frames
}


def srgb_to_linear(x):
    return np.where(x <= 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)


def linear_to_srgb(x):
    return np.where(x <= 0.0031308, x * 12.92, 1.055 * np.power(np.clip(x, 0, None), 1 / 2.4) - 0.055)


class SensorModel:
    def __init__(self, calib: dict):
        self.c = {**DEFAULT, **calib}
        self._fp_cache = {}

    @classmethod
    def from_json(cls, path):
        return cls(json.load(open(path)))

    # ---- fixed pattern maps (sensor property, cached per image size)
    def _fixed_pattern(self, shape):
        key = tuple(shape[:2])
        if key not in self._fp_cache:
            rng = np.random.default_rng(int(self.c["fixed_pattern_seed"]))
            prnu = 1.0 + rng.normal(0, self.c["prnu"], key).astype(np.float32)
            dsnu = rng.normal(0, self.c["dsnu_e"], key).astype(np.float32)
            self._fp_cache[key] = (prnu, dsnu)
        return self._fp_cache[key]

    def _vignette(self, shape):
        v = self.c["vignetting"]
        if not v:
            return 1.0
        h, w = shape[:2]
        cy, cx = v["center"][1] * h, v["center"][0] * w
        y, x = np.mgrid[0:h, 0:w].astype(np.float32)
        r2 = ((x - cx) / (w / 2)) ** 2 + ((y - cy) / (h / 2)) ** 2
        r2 = r2 / (1.0 + (h / w) ** 2)  # normalise so the corner is r = 1
        return np.clip(1.0 - v["k1"] * r2 - v["k2"] * r2**2, 0.05, 1.0)[..., None]

    def sample_params(self, rng):
        c = self.c
        lo, hi = c["exposure_s_range"]
        p = {
            "exposure_s": float(np.exp(rng.uniform(np.log(lo), np.log(hi)))),
            "ev_jitter": float(rng.normal(0, c["exposure_ev_sigma"])),
            "temperature_c": float(rng.uniform(*c["temperature_c_range"])),
            "defocus_sigma_px": float(rng.uniform(*c["defocus_sigma_px_range"])),
            "motion_blur_px": float(rng.uniform(*c["motion_blur_px_range"]))
            if rng.uniform() < c["motion_blur_prob"]
            else 0.0,
            "motion_angle_deg": float(rng.uniform(0, 180)),
        }
        return p

    def __call__(self, img_uint8, seed=None, params=None, return_params=False):
        """img_uint8: HxWx3 (RGB or BGR, treated per channel) or HxW. Returns the same shape, uint8."""
        c = self.c
        rng = np.random.default_rng(seed)
        p = params or self.sample_params(rng)
        gray = img_uint8.ndim == 2
        x = img_uint8[..., None] if gray else img_uint8
        lin = srgb_to_linear(x.astype(np.float32) / 255.0)
        # exposure: time relative to the reference, plus auto-exposure jitter in stops
        lin = lin * (p["exposure_s"] / c["exposure_ref_s"]) * (2.0 ** p["ev_jitter"])
        # optics
        lin = lin * self._vignette(lin.shape)
        if p["defocus_sigma_px"] > 0.05:
            lin = cv2.GaussianBlur(lin, (0, 0), p["defocus_sigma_px"]).reshape(lin.shape)
        if p["motion_blur_px"] >= 1.0:
            k = int(round(p["motion_blur_px"])) | 1
            ker = np.zeros((k, k), np.float32)
            ker[k // 2, :] = 1.0 / k
            M = cv2.getRotationMatrix2D((k / 2 - 0.5, k / 2 - 0.5), p["motion_angle_deg"], 1.0)
            ker = cv2.warpAffine(ker, M, (k, k))
            ker /= max(ker.sum(), 1e-6)
            lin = cv2.filter2D(lin, -1, ker).reshape(lin.shape)
        # sensor
        prnu, dsnu = self._fixed_pattern(lin.shape)
        e = np.clip(lin, 0, None) * c["full_well_e"] * prnu[..., None]
        e = rng.poisson(e).astype(np.float32)
        dark_rate = c["dark_current_e_per_s"] * 2.0 ** ((p["temperature_c"] - c["temp_ref_c"]) / c["temp_doubling_c"])
        dark_e = dark_rate * p["exposure_s"]
        e += (
            rng.poisson(np.full(e.shape[:2], dark_e, np.float32))[..., None]
            + (dsnu * (p["exposure_s"] / c["exposure_ref_s"]))[..., None]
        )
        e += rng.normal(0, c["read_noise_e"], e.shape).astype(np.float32)
        dn = e / c["gain_e_per_dn"] + c["black_level_dn"]
        dn_max = 2 ** c["bit_depth"] - 1
        dn = np.clip(np.round(dn), 0, dn_max)
        # back to the render's 8-bit sRGB convention: black level removed, full well -> 1.0
        out = (dn - c["black_level_dn"]) / (c["full_well_e"] / c["gain_e_per_dn"])
        out = np.clip(linear_to_srgb(np.clip(out, 0, 1)) * 255.0 + 0.5, 0, 255).astype(np.uint8)
        out = out[..., 0] if gray else out
        return (out, p) if return_params else out


def frame_seed(fid: str) -> int:
    """Deterministic per-frame seed (the published reference dataset's images_sensor set used the same rule)."""
    return zlib.crc32(fid.encode()) & 0xFFFFFFFF


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=None, help="JSON from sensor_calibrate.py (default: generic CMOS)")
    ap.add_argument("--in", dest="inp", required=True, help="folder of <fid>_L.png / <fid>_R.png (or any *.png)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args(argv)
    sm = SensorModel.from_json(a.calib) if a.calib else SensorModel({})
    src = Path(a.inp)
    files = sorted(src.glob("*.png")) if src.is_dir() else [src]
    groups = {}
    for f in files:  # a stereo pair shares one draw (exposure, temperature, blur) like a real rig; noise differs
        fid, side = (f.stem[:-2], f.stem[-1]) if f.stem[-2:] in ("_L", "_R") else (f.stem, "L")
        groups.setdefault(fid, []).append((side, f))
    fids = sorted(groups)[: a.limit or None]
    Path(a.out).mkdir(parents=True, exist_ok=True)
    log = {}
    for fid in fids:
        seed = frame_seed(fid)
        p = sm.sample_params(np.random.default_rng(seed))
        for side, f in sorted(groups[fid]):
            img = cv2.imread(str(f), cv2.IMREAD_UNCHANGED)
            out = sm(img, seed=seed * 2 + (0 if side == "L" else 1), params=p)
            cv2.imwrite(str(Path(a.out) / f.name), out, [cv2.IMWRITE_PNG_COMPRESSION, 4])
        log[fid] = {"seed": seed, **p}
    json.dump({"calib": sm.c, "frames": log}, open(Path(a.out) / "_sensor_params.json", "w"), indent=1)
    print(f"{sum(len(groups[f]) for f in fids)} images -> {a.out} (params in _sensor_params.json)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
