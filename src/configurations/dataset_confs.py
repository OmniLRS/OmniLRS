__author__ = "Sam S. Yu"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"

import copy
import dataclasses

DEFAULTS = {
    "rig": {
        "baseline_m": 0.12,
        "hfov_deg": 90.0,
        "fx": None,
        "fy": None,
        "cx": None,
        "cy": None,
        "height_m": [0.35, 1.0],
        "pitch_deg": [5.0, 22.0],
        "roll_jitter_deg": 2.0,
        "terrain_margin_m": 1.5,
    },
    "sun": {
        "elevation_buckets": [
            {"weight": 0.15, "range": [1.0, 5.0]},
            {"weight": 0.25, "range": [5.0, 10.0]},
            {"weight": 0.35, "range": [10.0, 25.0]},
            {"weight": 0.25, "range": [25.0, 60.0]},
        ],
        "intensity_mode": "constant_ground_brightness",
        "intensity": [1400.0, 2100.0],
        "intensity_base": 364.0,
        "intensity_jitter": 0.2,
        "temperature_k": [5600.0, 6500.0],
    },
    "largescale": {
        "region_radius_m": 150.0,
        "step_m": [40.0, 80.0],
        "frame_jitter_m": 8.0,
        "rock_clearance_m": 0.6,
        "dem_crop_half_m": 80.0,
        "dem_crop_res_m": 0.05,
    },
    "guards": {
        "dark_frame": {
            "enabled": False,
            "min_lit_fraction": 0.5,
            "retries": 3,
            "retry_min_elevation_deg": 10.0,
        },
        "auto_exposure": {"enabled": False, "min_mean": 70.0, "max_mean": 185.0, "target_mean": 125.0},
        "mesh_probe": {"enabled": False, "tol_m": 0.3},
        "hide_far_mesh": False,
        "pt_runtime_switch": False,
    },
}


def _merge(default: dict, user: dict, path: str) -> dict:
    """Deep-merge user over default. A dict or bool default only accepts a value of the same type, so a typo such as
    guards.mesh_probe: true fails here instead of after Isaac Sim has booted."""
    assert user is None or isinstance(user, dict), f"{path}: expected a dict, got {type(user).__name__} {user!r}"
    out = copy.deepcopy(default)
    for k, v in (user or {}).items():
        assert k in default, f"{path}.{k}: unknown_key {k!r}; valid keys: {sorted(default)}"
        if isinstance(default[k], dict):
            assert isinstance(v, dict), f"{path}.{k}: expected a dict, got {type(v).__name__} {v!r}"
            out[k] = _merge(default[k], v, f"{path}.{k}")
        else:
            if isinstance(default[k], bool):
                assert isinstance(v, bool), f"{path}.{k}: expected a bool, got {type(v).__name__} {v!r}"
            out[k] = v
    return out


def _range(r, name: str) -> None:
    assert len(r) == 2 and float(r[0]) <= float(r[1]), f"{name} must be [lo, hi] with lo <= hi, got {r}"


@dataclasses.dataclass
class DatasetConf:
    """
    Settings for mode=SDG dataset=<preset> (see docs/sdg_dataset.md).

    Args:
        base_seed (int): terrain k of this shard uses seed base_seed * 1000 + k.
        num_terrains (int): terrains (Lunaryard) or locations (LargeScale) per shard, at most 1000.
        frames_per_terrain (int): frames recorded per terrain/location.
        out_dir (str): shards are written to <out_dir>/shard_<base_seed:05d>/.
        settle_steps (int): render steps after every re-roll, before recording.
        exit_watchdog_s (float): if set, force the process to exit this many seconds after the run ends.
        rig (dict): camera rig; see DEFAULTS["rig"].
        sun (dict): sun sampling; see DEFAULTS["sun"].
        largescale (dict): LargeScale location walk; see DEFAULTS["largescale"].
        guards (dict): opt-in quality guards; see DEFAULTS["guards"].
        terrain_material (dict): optional {texture_scale, bump_factor} applied to the terrain shader.
    """

    base_seed: int = 0
    num_terrains: int = 10
    frames_per_terrain: int = 25
    out_dir: str = "data/sdg_dataset"
    settle_steps: int = 4
    exit_watchdog_s: float = None
    rig: dict = dataclasses.field(default_factory=dict)
    sun: dict = dataclasses.field(default_factory=dict)
    largescale: dict = dataclasses.field(default_factory=dict)
    guards: dict = dataclasses.field(default_factory=dict)
    terrain_material: dict = None

    def __post_init__(self):
        for name in ("rig", "sun", "largescale", "guards"):
            setattr(self, name, _merge(DEFAULTS[name], getattr(self, name), name))
        assert int(self.base_seed) >= 0, "base_seed must be >= 0"
        # terrain_seed = base_seed * 1000 + k: more than 1000 terrains would reuse the next shard's seeds
        assert 1 <= int(self.num_terrains) <= 1000, f"num_terrains must be in [1, 1000], got {self.num_terrains}"
        assert int(self.frames_per_terrain) >= 1, "frames_per_terrain must be >= 1"
        assert int(self.settle_steps) >= 0, "settle_steps must be >= 0"
        assert self.exit_watchdog_s is None or float(self.exit_watchdog_s) > 0, "exit_watchdog_s must be > 0 or null"
        r = self.rig
        assert float(r["baseline_m"]) >= 0, "rig.baseline_m must be >= 0"
        assert r["fx"] is not None or 0 < float(r["hfov_deg"]) < 180, "rig.hfov_deg must be in (0, 180)"
        for k in ("height_m", "pitch_deg"):
            _range(r[k], f"rig.{k}")
        s = self.sun
        assert s["intensity_mode"] in ("constant_ground_brightness", "range"), (
            f"sun.intensity_mode must be constant_ground_brightness or range, got {s['intensity_mode']!r}"
        )
        assert len(s["elevation_buckets"]) >= 1, "sun.elevation_buckets must not be empty"
        for b in s["elevation_buckets"]:
            assert float(b["weight"]) > 0, f"sun.elevation_buckets weight must be > 0, got {b}"
            _range(b["range"], "sun.elevation_buckets range")
        for k in ("intensity", "temperature_k"):
            _range(s[k], f"sun.{k}")
        _range(self.largescale["step_m"], "largescale.step_m")
        if self.terrain_material is not None:
            assert set(self.terrain_material) <= {"texture_scale", "bump_factor"}, (
                "terrain_material keys: texture_scale, bump_factor"
            )

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)
