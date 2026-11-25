"""Pure helpers of mode=SDG dataset=<preset> (no Isaac Sim)."""

import numpy as np
import pytest

from src.configurations.dataset_confs import DatasetConf
from src.environments_wrappers.sdg.dataset import guards, sampling


def test_intrinsics_match_dataset_camera():
    # published reference dataset (docs/sdg_dataset.md, 'Reproducing') cam_left_intrinsics.json:
    # 1640x1232, hfov 90 -> K = [[820,0,820],[0,820,616],[0,0,1]]
    k = sampling.compute_intrinsics(1640, 1232, DatasetConf().rig)
    assert np.allclose(k["K"], [[820, 0, 820], [0, 820, 616], [0, 0, 1]])
    assert k["usd"]["horizontal_aperture_mm"] == pytest.approx(48.0)
    assert k["usd"]["vertical_aperture_mm"] == pytest.approx(36.05853658536585)


def test_intrinsics_from_calibration_override_hfov():
    rig = DatasetConf(rig={"fx": 700.0, "fy": 710.0, "cx": 650.0, "cy": 470.0}).rig
    k = sampling.compute_intrinsics(1280, 960, rig)
    assert np.allclose(k["K"], [[700, 0, 650], [0, 710, 470], [0, 0, 1]])
    assert k["usd"]["horizontal_aperture_offset_mm"] != 0.0


def test_camera_offsets_mono_and_stereo():
    assert sampling.camera_offsets(["cam"], 0.12) == {"cam": 0.0}
    assert sampling.camera_offsets(["l", "r"], 0.12) == {"l": 0.06, "r": -0.06}
    with pytest.raises(ValueError):
        sampling.camera_offsets(["a", "b", "c"], 0.12)


def test_sun_sampling_is_deterministic_and_in_range():
    sun = DatasetConf().sun
    a = [sampling.sample_sun(np.random.default_rng(7), sun) for _ in range(3)]
    b = [sampling.sample_sun(np.random.default_rng(7), sun) for _ in range(3)]
    assert a == b
    rng = np.random.default_rng(0)
    el = [sampling.sample_sun(rng, sun)["elevation_deg"] for _ in range(2000)]
    assert 1.0 <= min(el) and max(el) <= 60.0
    assert 0.3 < np.mean(np.array(el) < 10.0) < 0.5  # ~40 % under 10 deg by construction


def test_sun_min_elevation_respected():
    rng = np.random.default_rng(1)
    assert all(sampling.sample_sun(rng, DatasetConf().sun, 10.0)["elevation_deg"] >= 10.0 for _ in range(200))


def test_sun_and_rig_draws_are_pinned():
    # golden values: a seed must keep meaning the same frames, so any reordering of the rng draws (sun: bucket,
    # elevation, azimuth, intensity jitter, temperature; rig: height, yaw, pitch, roll) fails here
    rng = np.random.default_rng(12345)
    sun = sampling.sample_sun(rng, DatasetConf().sun)
    assert sun == {"elevation_deg": 6.584, "azimuth_deg": 287.052, "intensity": 3398.5, "temperature_k": 5952.0}
    rec = sampling.sample_rig_pose(rng, DatasetConf().rig, 1.5, -2.0, 0.25)
    assert {k: v for k, v in rec.items() if k != "quat_xyzw"} == {
        "position": [1.5, -2.0, 0.8163],
        "height_above_ground_m": 0.5663,
        "yaw_deg": 215.391,
        "pitch_deg": 8.174,
        "roll_deg": 0.691,
    }
    assert rec["quat_xyzw"] == pytest.approx([-0.06973, -0.015934, 0.950375, -0.302771], abs=2e-6)
    ranged = sampling.sample_sun(np.random.default_rng(12345), DatasetConf(sun={"intensity_mode": "range"}).sun)
    assert ranged == {"elevation_deg": 6.584, "azimuth_deg": 287.052, "intensity": 1873.4, "temperature_k": 5952.0}


def test_rig_pose_height_above_ground():
    rig = DatasetConf().rig
    rec = sampling.sample_rig_pose(np.random.default_rng(3), rig, 1.0, 2.0, ground_z=-5.0)
    assert 0.35 <= rec["height_above_ground_m"] <= 1.0
    assert rec["position"][2] == pytest.approx(-5.0 + rec["height_above_ground_m"], abs=1e-3)
    assert np.linalg.norm(rec["quat_xyzw"]) == pytest.approx(1.0, abs=1e-5)


def test_walk_stays_in_region():
    rng = np.random.default_rng(5)
    xy = np.zeros(2)
    for k in range(200):
        xy = sampling.next_walk_location(rng, xy, k, 150.0, [40.0, 80.0])
        assert np.linalg.norm(xy) <= 150.0 + 1e-9


def test_lit_stats_and_gain():
    depth = np.full((100, 100), 5.0, np.float32)
    rgb = np.zeros((100, 100, 3), np.uint8)
    rgb[:, :50] = 200
    lit, mean = guards.lit_stats(rgb, depth)
    assert lit == pytest.approx(0.5) and mean == pytest.approx(200.0)
    ae = DatasetConf().guards["auto_exposure"]
    assert guards.exposure_gain(100.0, ae) is None
    assert guards.exposure_gain(250.0, ae) == pytest.approx(0.5)
    assert guards.exposure_gain(1.0, ae) == pytest.approx(4.0)


def test_probe_error_zero_on_consistent_ground():
    # camera 1 m above flat ground z=0, looking straight down (pitch 90)
    K = np.array([[100.0, 0, 50], [0, 100.0, 50], [0, 0, 1]])
    rig = {"position": [0.0, 0.0, 1.0], "quat_xyzw": [0.0, 0.7071068, 0.0, 0.7071068]}
    depth = np.full((100, 100), 1.0, np.float32)
    err = guards.probe_error(depth, K, rig, 0.0, lambda x, y: 0.0)
    assert err == pytest.approx(0.0, abs=1e-3)
    assert guards.probe_error(depth, K, rig, 0.0, lambda x, y: 0.5) == pytest.approx(0.5, abs=1e-3)
