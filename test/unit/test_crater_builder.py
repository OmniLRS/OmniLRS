"""CraterBuilder adds to each block DEM only the part of a crater stamp that overlaps the block.

A crater crossing the edge between two blocks is then continuous across them.

python -m pytest -v test/unit/test_crater_builder.py
"""

import importlib.util
import sys
from importlib.machinery import ModuleSpec
from unittest.mock import MagicMock

import numpy as np
import pytest

pytest.importorskip("scipy")
# utils imports zfpy for the rock data only; the crater code never calls it. The stand-in gets a spec, as in
# test_crater_metadata_region.py, so the find_spec checks of the other test modules accept it.
if "zfpy" not in sys.modules and importlib.util.find_spec("zfpy") is None:
    sys.modules["zfpy"] = MagicMock(__spec__=ModuleSpec("zfpy", None))

from src.terrain_management.large_scale_terrain.crater_generation import (  # noqa: E402
    CraterBuilder,
    CraterBuilderConf,
)
from src.terrain_management.large_scale_terrain.utils import CraterMetadata  # noqa: E402

RES = 0.025  # m/px, as in the LargeScale configs
BLOCK = 50
PAD = 10  # 2 * PAD < 2 * RADIUS: the former padded canvas could not hold the whole stamp
RADIUS = 6.0


class FakeDB:
    """Round crater with a parabolic bowl, no deformation and no marks."""

    def get_deformation_spline(self, _id):
        return lambda t: np.ones_like(t)

    def get_marks_spline(self, _id):
        return lambda t: np.zeros_like(t)

    def get_crater_profile_spline(self, _id):
        return lambda t: t**2 - 1.0  # -1 at the centre, 0 at the rim and beyond


def _builder(block_size=BLOCK):
    conf = CraterBuilderConf(block_size=block_size, pad_size=PAD, resolution=RES, z_scale=1.0)
    return CraterBuilder(conf, db=FakeDB())


def _crater(x, y):
    return CraterMetadata(
        radius=RADIUS,
        coordinates=(x, y),
        marks_intensity=0.0,
        xy_deformation_factor=(1.0, 1.0),
        rotation=0.0,
    )


# Centres 5 m past the shared edge of two blocks, on either side of x = 0 (one centre at negative block pixels).
# Half a pixel off the grid, so float rounding of the block-relative coordinates cannot move the centre pixel.
@pytest.mark.parametrize(
    "x_center, first_block_x",
    [(BLOCK + 5 + RES / 2, 0), (-5 + RES / 2, -BLOCK)],
)
def test_crater_straddling_a_block_edge_is_continuous(x_center, first_block_x):
    crater = [_crater(x_center, 25 + RES / 2)]
    first = _builder().generate_craters(crater, (first_block_x, 0))
    second = _builder().generate_craters(crater, (first_block_x + BLOCK, 0))
    # One block covering both: the whole stamp fits, nothing is clipped
    both = _builder(block_size=2 * BLOCK).generate_craters(crater, (first_block_x, 0))[:, : first.shape[1]]

    assert first.shape == second.shape == (2000, 2000)
    assert np.any(first != 0) and np.any(second != 0), "the crater must reach into both blocks"
    np.testing.assert_array_equal(np.concatenate([first, second], axis=0), both)
