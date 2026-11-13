"""Large scale terrain cache (MapManagerConf.terrain_cache_dir): a start restored from the cache must stream the same
terrain as the start that generated it, and the cache key must only depend on what the terrain depends on."""

import pickle
import signal
from unittest.mock import patch

import numpy as np
import pytest
from scipy.interpolate import CubicSpline

from src.terrain_management.large_scale_terrain.crater_distribution import CraterMetadataGenerator
from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import HighResDEMGen
from src.terrain_management.large_scale_terrain.map_manager import MapManager, MapManagerConf

START = (-10.0, 10.0)


@pytest.fixture
def make_map_manager(tmp_path):
    """Builds map managers on a small source DEM and HR window (3x3 blocks of 10 m), sharing one cache folder."""

    source = tmp_path / "dems" / "test"
    source.mkdir(parents=True)
    np.save(source / "dem.npy", np.arange(3600, dtype=np.float32).reshape(60, 60))
    (source / "dem.yaml").write_text("size: [60, 60]\npixel_size: [5.0, 5.0]\ncenter_coordinates: [0.0, 0.0]\n")
    profiles_path = tmp_path / "crater_profiles.pkl"
    profiles_path.write_bytes(pickle.dumps([CubicSpline([0.0, 0.5, 1.0], [-1.0, -0.2, 0.0])]))
    workers = {"num_workers": 1, "input_queue_size": 64, "output_queue_size": 64}
    hrdem_settings = {
        "high_res_dem_cfg": {
            "num_blocks": 1,
            "block_size": 10,
            "pad_size": 2.0,
            "max_blocks": 100,
            "seed": 42,
            "resolution": 0.1,
            "z_scale": 1.0,
            "source_resolution": 5.0,
            "interpolation_padding": 2,
            "generate_craters": True,
        },
        "crater_db_cfg": {"block_size": 10},
        "crater_sampler_cfg": {
            "block_size": 10,
            "crater_gen_cfg": {
                "profiles_path": str(profiles_path),
                "num_unique_profiles": 8,
                "seed": 43,
                "min_xy_ratio": 0.85,
                "max_xy_ratio": 1.0,
                "random_rotation": True,
            },
            "crater_dist_cfg": {"densities": [0.01], "radius": [[0.5, 0.75]], "seed": 44},
        },
        "crater_builder_cfg": {"block_size": 10, "pad_size": 2.0, "resolution": 0.1, "z_scale": 1.0},
        "interpolator_cfg": {
            "source_resolution": 5.0,
            "target_resolution": 0.1,
            "source_padding": 2,
            "method": "bicubic",
        },
        "crater_worker_manager_cfg": workers,
        "interpolator_worker_manager_cfg": workers,
    }
    managers = []
    previous_sigint = signal.getsignal(signal.SIGINT)  # ThreadMonitor replaces the SIGINT handler

    def make():
        manager = MapManager(
            MapManagerConf(
                folder_path=str(source.parent), hrdem_settings=hrdem_settings, terrain_cache_dir=str(tmp_path / "cache")
            )
        )
        manager.load_lr_dem_by_name("test")
        managers.append(manager)
        return manager

    yield make
    for manager in managers:
        if hasattr(manager, "hr_dem_gen"):
            # The monitor thread shuts the workers down once its event is set.
            manager.hr_dem_gen.monitor_thread.event.set()
            manager.hr_dem_gen.monitor_thread.thread.join(timeout=10)
    signal.signal(signal.SIGINT, previous_sigint)


def test_cached_terrain_is_loaded_and_streams_like_the_generated_one(make_map_manager):
    generated = make_map_manager()
    # Before initializing: building the generator converts interpolator_cfg.method to a cv2/PIL constant in place.
    cache_path = generated.get_terrain_cache_path(START)
    generated.initialize_hr_dem(START)
    assert (cache_path / "dem.npy").is_file()

    restored = make_map_manager()
    with (
        patch.object(HighResDEMGen, "shift", side_effect=AssertionError("generated a cached terrain")),
        patch.object(CraterMetadataGenerator, "build", side_effect=AssertionError("generated cached profiles")),
    ):
        restored.initialize_hr_dem(START)
    np.testing.assert_array_equal(generated.get_hr_dem(), restored.get_hr_dem())

    # New blocks get new craters: they only match if the crater sampler resumed from the cached state.
    for position in ((0.0, 10.0), (-20.0, 0.0)):
        generated.hr_dem_gen.update_terrain_data_blocking(position)
        restored.hr_dem_gen.update_terrain_data_blocking(position)
        np.testing.assert_array_equal(generated.get_hr_dem(), restored.get_hr_dem())


def test_cache_path_only_depends_on_what_the_terrain_depends_on(make_map_manager):
    manager = make_map_manager()
    path = manager.get_terrain_cache_path(START)

    manager.hr_dem_settings.crater_worker_manager_cfg.num_workers += 1
    assert manager.get_terrain_cache_path(START) == path
    assert manager.get_terrain_cache_path((0.0, 0.0)) != path
    manager.hr_dem_settings.high_res_dem_cfg.seed += 1
    assert manager.get_terrain_cache_path(START) != path
    manager.settings.terrain_cache_dir = None
    assert manager.get_terrain_cache_path(START) is None
