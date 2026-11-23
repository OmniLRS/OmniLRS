"""Large scale crater metadata region: the crater builder adds to each block of the high resolution grid the craters of
the block and of its 8 neighbours that overlap it, so they must be in the crater DB when the block is dispatched.
Otherwise the craters that reach in from a missing neighbour are cut at the block edge, and the block is never rebuilt.

Runs the real HighResDEMGen grid and metadata methods, CraterDB and CraterSampler on a stand-in object: no workers."""

import importlib.util
import pickle
import sys
from importlib.machinery import ModuleSpec
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from scipy.interpolate import CubicSpline

# The terrain modules import zfpy (rocks) and matplotlib (debug plots) at module level, but this test does not use
# them. They are not in the test-unit environment (see test_terrain_cache.py). The stand-ins get a spec, so the
# find_spec checks of the other test modules accept them whatever the collection order.
for module in ("zfpy", "matplotlib"):
    if module not in sys.modules and importlib.util.find_spec(module) is None:
        sys.modules[module] = MagicMock(__spec__=ModuleSpec(module, None))

from src.terrain_management.large_scale_terrain.crater_distribution import (  # noqa: E402
    CraterSampler,
    CraterSamplerConf,
)
from src.terrain_management.large_scale_terrain.crater_generation import CraterDB, CraterDBConf  # noqa: E402
from src.terrain_management.large_scale_terrain.high_resolution_DEM_generator import (  # noqa: E402
    HighResDEMConf,
    HighResDEMGen,
)

BLOCK_SIZE = 10


@pytest.fixture
def make_generator(tmp_path):
    profiles_path = tmp_path / "crater_profiles.pkl"
    profiles_path.write_bytes(pickle.dumps([CubicSpline([0.0, 0.5, 1.0], [-1.0, -0.2, 0.0])]))

    def make(num_blocks, center):
        db = CraterDB(CraterDBConf(block_size=BLOCK_SIZE))
        sampler_cfg = CraterSamplerConf(
            block_size=BLOCK_SIZE,
            crater_gen_cfg={
                "profiles_path": str(profiles_path),
                "num_unique_profiles": 8,
                "seed": 43,
                "min_xy_ratio": 0.85,
                "max_xy_ratio": 1.0,
                "random_rotation": True,
            },
            crater_dist_cfg={"densities": [0.01], "radius": [[0.5, 0.75]], "seed": 44},
        )
        settings = HighResDEMConf(
            num_blocks=num_blocks,
            block_size=BLOCK_SIZE,
            pad_size=2.0,
            max_blocks=100,
            seed=42,
            resolution=0.1,
            z_scale=1.0,
            source_resolution=5.0,
            interpolation_padding=2,
            generate_craters=True,
        )
        gen = SimpleNamespace(
            settings=settings,
            current_block_coord=center,
            crater_db=db,
            crater_sampler=CraterSampler(sampler_cfg, db=db),
        )
        HighResDEMGen.build_block_grid(gen)
        return gen

    return make


def missing_neighbours(gen) -> dict:
    """Grid block offset from the centre (in blocks) -> neighbours absent from the crater DB."""

    missing = {}
    for x, y in gen.map_grid_block2coords:
        absent = [
            (x + dx, y + dy)
            for dx in (-BLOCK_SIZE, 0, BLOCK_SIZE)
            for dy in (-BLOCK_SIZE, 0, BLOCK_SIZE)
            if not gen.crater_db.check_block_exists((x + dx, y + dy))
        ]
        if absent:
            offset = ((x - gen.current_block_coord[0]) // BLOCK_SIZE, (y - gen.current_block_coord[1]) // BLOCK_SIZE)
            missing[offset] = absent
    return missing


@pytest.mark.parametrize("num_blocks", [1, 4])
@pytest.mark.parametrize("center", [(0, 0), (-30, 20)])
def test_every_grid_block_has_its_crater_neighbours(make_generator, num_blocks, center):
    gen = make_generator(num_blocks, center)
    HighResDEMGen.generate_craters_metadata(gen, center)
    assert all(state["has_crater_metadata"] for state in gen.block_grid_tracker.values())
    assert missing_neighbours(gen) == {}


def test_grid_blocks_keep_their_crater_neighbours_after_shifts(make_generator):
    # Same sequence as HighResDEMGen.shift: shift the grid, move the centre, then generate the crater metadata.
    gen = make_generator(2, (0, 0))
    HighResDEMGen.generate_craters_metadata(gen, (0, 0))
    for step in ((1, 0), (0, 1), (-1, -1), (3, 2)):
        center = (gen.current_block_coord[0] + step[0] * BLOCK_SIZE, gen.current_block_coord[1] + step[1] * BLOCK_SIZE)
        HighResDEMGen.shift_block_grid(gen, center)
        gen.current_block_coord = center
        HighResDEMGen.generate_craters_metadata(gen, center)
        assert missing_neighbours(gen) == {}, f"after the shift to {center}"
