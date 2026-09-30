"""Offline dataset tools (scripts/sdg_dataset). No Isaac Sim required."""

__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import json

import cv2
import numpy as np
import pytest
from sdg_fixtures import make_shard

from scripts.sdg_dataset import _common, build, flag_frames, sensor_model, validate


def _sem(out, side="L"):
    return cv2.imread(str(out / "shard_00007" / "labels" / f"s00007_t0000_f000_{side}_sem.png"), cv2.IMREAD_UNCHANGED)


def test_rock_size_is_measured_from_depth(tmp_path):
    shard = make_shard(tmp_path)
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    sem = _sem(out)
    assert sem[22, 32] == _common.SEM["rock_large"]  # 10 cm >= 5 cm clearance
    assert sem[42, 14] == _common.SEM["rock_small"]  # 3 cm < 5 cm
    assert sem[5, 5] == _common.SEM["ground"]


def test_measured_height_beats_the_rock_table(tmp_path):
    # the table claims 3 cm / 10 cm; the depth says 10 cm / 3 cm. The depth wins.
    shard = make_shard(tmp_path, table_heights=(0.03, 0.10))
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    sem = _sem(out)
    assert sem[22, 32] == _common.SEM["rock_large"]
    assert sem[42, 14] == _common.SEM["rock_small"]


def test_unmeasurable_blob_takes_its_majority_table_class(tmp_path):
    # two touching instances at 70 m (beyond the 65 m measuring range): 6x6 table-large + 3x3 table-small
    rocks = ((0.10, 20, 30, 70.0, 6), (0.03, 20, 36, 70.0, 3))
    shard = make_shard(tmp_path, rocks=rocks, table_heights=(0.10, 0.03))
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    sem = _sem(out)
    assert (sem[20:26, 30:36] == _common.SEM["rock_large"]).all()
    assert (sem[20:23, 36:39] == _common.SEM["rock_large"]).all()  # minority pixels follow the blob majority
    meta = json.loads((out / "shard_00007" / "meta" / "s00007_t0000_f000.json").read_text())
    (blob,) = meta["cams"]["L"]["rocks_measured"]
    assert blob["method"] == "far" and blob["height_m"] is None and blob["class"] == "rock_large"


@pytest.mark.parametrize("suffix", ["", "/mesh"])
def test_rock_table_join_does_not_match_a_path_prefix(tmp_path, suffix):
    # instance_1 is a string prefix of instance_10; each blob must still get its own table entry. Both blobs sit
    # at 70 m (unmeasurable), so the table class is what ends up in the label.
    rocks = ((0.03, 10, 10, 70.0), (0.10, 30, 40, 70.0))
    paths = ("/World/Rocks/instance_1", "/World/Rocks/instance_10")
    shard = make_shard(tmp_path, rocks=rocks, rock_paths=paths, label_suffix=suffix)
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    sem = _sem(out)
    assert (sem[10:16, 10:16] == _common.SEM["rock_small"]).all()
    assert (sem[30:36, 40:46] == _common.SEM["rock_large"]).all()
    meta = json.loads((out / "shard_00007" / "meta" / "s00007_t0000_f000.json").read_text())
    rocks_seen = meta["cams"]["L"]["rocks"]
    assert set(rocks_seen) == set(paths)
    assert rocks_seen[paths[1]]["class"] == "rock_large"


def test_clearance_flag_changes_the_split(tmp_path):
    shard = make_shard(tmp_path)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1", "--wheel-clearance-m", "0.2"])
    assert _sem(out)[22, 32] == _common.SEM["rock_small"]


def test_trav_and_bits(tmp_path):
    shard = make_shard(tmp_path)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"])
    base = out / "shard_00007"
    trav = cv2.imread(str(base / "labels" / "s00007_t0000_f000_L_trav.png"), cv2.IMREAD_UNCHANGED)
    bits = cv2.imread(str(base / "masks" / "s00007_t0000_f000_L_bits.png"), cv2.IMREAD_UNCHANGED)
    assert trav[22, 32] == 2 and trav[42, 14] == 1 and trav[5, 5] == 0
    assert bits[22, 32] & _common.BIT["rock_large"] and bits[42, 14] & _common.BIT["rock_small"]
    assert (base / "terrains" / "terrain_0000.npz").exists()


def test_crater_bit_from_dem(tmp_path):
    shard = make_shard(tmp_path, rocks=(), crater=True)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"])
    bits = cv2.imread(str(out / "shard_00007" / "masks" / "s00007_t0000_f000_L_bits.png"), cv2.IMREAD_UNCHANGED)
    assert (bits & _common.BIT["crater"]).any()


def test_slope_bits_from_dem(tmp_path):
    shard = make_shard(tmp_path, rocks=(), dem_tilt_deg=30.0)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"])
    bits = cv2.imread(str(out / "shard_00007" / "masks" / "s00007_t0000_f000_L_bits.png"), cv2.IMREAD_UNCHANGED)
    assert (bits[10:-10, 10:-10] & _common.BIT["slope_hazard"]).all()


def test_frame_without_rocks(tmp_path):
    shard = make_shard(tmp_path, rocks=())
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    assert set(np.unique(_sem(out))) == {_common.SEM["ground"]}


def test_mono_rig(tmp_path):
    shard = make_shard(tmp_path, cameras=("cam_left",))
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    assert not (out / "shard_00007" / "labels" / "s00007_t0000_f000_R_sem.png").exists()


def test_partial_shard_builds_what_exists(tmp_path):
    shard = make_shard(tmp_path, frames=3, drop_files_for=(1,))
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"]) == 0
    summary = json.loads((out / "summary.json").read_text())
    assert summary["frames"] == 2 and summary["missing"] == ["s00007_t0000_f001"]


def test_element_per_folder_from_the_manifest(tmp_path):
    # 3 frames at 2 per folder: files in 0/0.png, 0/1.png, 1/0.png
    shard = make_shard(tmp_path, frames=3, epf=2)
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1", "--sizes", "S=100"]) == 0
    summary = json.loads((out / "summary.json").read_text())
    assert summary["frames"] == 3 and summary["missing"] == []


def test_build_fails_when_no_frame_could_be_built(tmp_path):
    shard = make_shard(tmp_path, frames=2, drop_files_for=(0, 1))
    assert build.main(["--shards", str(shard), "--out", str(tmp_path / "out"), "--workers", "1"]) == 1


def test_missing_manifest_field_is_named(tmp_path):
    shard = make_shard(tmp_path)
    man = json.loads((shard / "manifest.json").read_text())
    del man["intrinsics"]
    (shard / "manifest.json").write_text(json.dumps(man))
    with pytest.raises(_common.ContractError, match="intrinsics"):
        build.main(["--shards", str(shard), "--out", str(tmp_path / "out"), "--workers", "1"])


def test_split_is_by_terrain_seed_and_matches_published_rule():
    # published reference dataset rule: md5(str(seed)) % 1000 % 100 < val_frac * 100 -> val
    assert {build.split_of(s, 0.15, 0.0) for s in range(1000, 1400)} == {"train", "val"}
    assert build.split_of(1234, 0.15, 0.0) == build.split_of(1234, 0.15, 0.0)


def test_exclude_list_and_splits_only(tmp_path):
    shard = make_shard(tmp_path, frames=2)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1", "--sizes", "S=100"])
    ex = tmp_path / "rejected.txt"
    ex.write_text("s00007_t0000_f000\trock_issue\n")
    build.main(["--out", str(out), "--splits-only", "--exclude", str(ex), "--sizes", "S=100"])
    ids = "".join(p.read_text() for p in (out / "splits").glob("S_*.txt"))
    assert "s00007_t0000_f000" not in ids and "s00007_t0000_f001" in ids


def test_generator_lit_fraction_under_guards_sends_frame_to_dark(tmp_path):
    shard = make_shard(tmp_path, frames=2)
    man = json.loads((shard / "manifest.json").read_text())
    man["frames"][0]["guards"] = {"lit_fraction": 0.2}
    (shard / "manifest.json").write_text(json.dumps(man))
    out = tmp_path / "out"
    assert build.main(["--shards", str(shard), "--out", str(out), "--workers", "1", "--sizes", "S=100"]) == 0
    dark = (out / "splits" / "S_dark.txt").read_text().split()
    assert dark == ["s00007_t0000_f000"]


def test_validate_passes_on_consistent_shard(tmp_path):
    shard = make_shard(tmp_path, frames=3)
    rc = validate.main([str(shard), "--n", "3"])
    rep = json.loads((shard / "validation_report.json").read_text())
    assert rc == 0, rep["fail"]


def test_validate_fails_on_missing_files(tmp_path):
    shard = make_shard(tmp_path, frames=3, drop_files_for=(1,))
    assert validate.main([str(shard), "--n", "2"]) == 1
    rep = json.loads((shard / "validation_report.json").read_text())
    assert any("files !=" in m and "frames" in m for m in rep["fail"]), rep["fail"]


def test_validate_reads_element_per_folder(tmp_path):
    shard = make_shard(tmp_path, frames=3, epf=2)
    rc = validate.main([str(shard), "--n", "3"])
    rep = json.loads((shard / "validation_report.json").read_text())
    assert rc == 0, rep["fail"]
    assert rep["frames_with_files"] == 3


def _darken_with_sun(shard):
    """Sun elevation rises frame by frame while the ground gets darker: the opposite of the expected correlation."""
    man = json.loads((shard / "manifest.json").read_text())
    data = next(p for p in shard.iterdir() if p.is_dir() and p.name != "terrains")
    for i, fr in enumerate(man["frames"]):
        fr["sun"]["elevation_deg"] = 5.0 + 5.0 * i
        for cam in man["intrinsics"]:
            cv2.imwrite(str(data / f"{cam}_rgb" / "0" / f"{i:04d}.png"), np.full((48, 64, 3), 200 - 10 * i, np.uint8))
    (shard / "manifest.json").write_text(json.dumps(man))


def test_validate_sun_check_only_warns_on_a_small_sample(tmp_path):
    shard = make_shard(tmp_path, frames=6)
    _darken_with_sun(shard)
    rc = validate.main([str(shard), "--n", "12"])
    rep = json.loads((shard / "validation_report.json").read_text())
    assert rc == 0, rep["fail"]
    assert any("sun elevation" in w for w in rep["warn"]), rep["warn"]


def test_validate_sun_check_fails_on_a_large_sample(tmp_path):
    shard = make_shard(tmp_path, frames=10)
    _darken_with_sun(shard)
    assert validate.main([str(shard), "--n", "12"]) == 1
    rep = json.loads((shard / "validation_report.json").read_text())
    assert any("sun elevation" in m for m in rep["fail"]), rep["fail"]


def test_validate_mono(tmp_path):
    shard = make_shard(tmp_path, frames=2, cameras=("cam_left",))
    assert validate.main([str(shard), "--n", "2"]) == 0


def test_flag_frames_writes_rejected_list(tmp_path):
    shard = make_shard(tmp_path, frames=2)
    out = tmp_path / "out"
    build.main(["--shards", str(shard), "--out", str(out), "--workers", "1"])
    assert flag_frames.main([str(out / "shard_00007"), "--workers", "1"]) == 0
    q = json.loads((out / "shard_00007" / "quality.json").read_text())
    assert set(q["frames"]) == {"s00007_t0000_f000", "s00007_t0000_f001"}
    assert (out / "shard_00007" / "rejected.txt").exists()
    for fid, fr in q["frames"].items():
        assert "error" not in fr, (fid, fr.get("error"))
        assert "near_lit_frac" in fr and isinstance(fr["far_flat_frac"], (int, float)), fid


def test_sensor_model_cli_is_deterministic(tmp_path):
    src = tmp_path / "images"
    src.mkdir()
    img = np.random.default_rng(0).integers(0, 255, (32, 48, 3), dtype=np.uint8)
    for side in ("L", "R"):
        cv2.imwrite(str(src / f"s00001_t0000_f000_{side}.png"), img)
    for run in ("a", "b"):
        assert sensor_model.main(["--in", str(src), "--out", str(tmp_path / run)]) == 0
    for side in ("L", "R"):
        a = cv2.imread(str(tmp_path / "a" / f"s00001_t0000_f000_{side}.png"))
        b = cv2.imread(str(tmp_path / "b" / f"s00001_t0000_f000_{side}.png"))
        assert a.dtype == np.uint8 and a.shape == img.shape and np.array_equal(a, b)
    params = json.loads((tmp_path / "a" / "_sensor_params.json").read_text())["frames"]
    assert params["s00001_t0000_f000"]["seed"] == sensor_model.frame_seed("s00001_t0000_f000")
