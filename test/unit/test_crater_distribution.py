"""Unit tests for the automatic crater bins and densities of src/configurations/procedural_terrain_confs.py
(no Isaac Sim).

python -m pytest -v test/unit/test_crater_distribution.py
"""

import math
import unittest

from src.configurations.procedural_terrain_confs import (
    CraterDistributionConf,
    compute_bin_densities,
    compute_radius_bins,
)


class TestCraterBinDensities(unittest.TestCase):
    def test_sampled_radius_is_used_as_rim_diameter(self):
        # The rim sits at about half the sampled radius, so the rim diameter equals the radius:
        # N(>=1 m) - N(>=2 m) = 100 - 25 = 75 per km^2.
        self.assertEqual(compute_bin_densities([[1.0, 2.0]], 100.0, -2.0), [75 / 1_000_000])

    def test_density_is_converted_from_per_km2_to_per_m2(self):
        # N(>=0.5 m) - N(>=1 m) = 400 - 100 = 300 per km^2.
        self.assertEqual(compute_bin_densities([[0.5, 1.0]], 100.0, -2.0), [0.0003])

    def test_nasa_example_initial_poisson_mean(self):
        densities = compute_bin_densities([[5.0, 5.5]], 76192.0, -2.132)

        # Rim diameter 5–5.5 m on 100 x 100 m: 2464.3666590 - 2011.2039798 craters per km^2.
        # This is a Poisson proposal mean, not a count after spatial rejection.
        self.assertTrue(math.isclose(10_000 * densities[0], 4.531626791923868, rel_tol=1e-12))

    def test_splitting_an_interval_preserves_its_total_density(self):
        partitions = [
            [[0.5, 2.5]],
            [[0.5, 1.0], [1.0, 1.5], [1.5, 2.0], [2.0, 2.5]],
            [[0.5, 0.75], [0.75, 2.0], [2.0, 2.5]],
        ]
        for radius_bins in partitions:
            with self.subTest(radius_bins=radius_bins):
                densities = compute_bin_densities(radius_bins, 100.0, -2.0)

                # Every partition covers rim diameters 0.5–2.5 m: N(>=0.5) - N(>=2.5) = 400 - 16.
                self.assertEqual(len(densities), len(radius_bins))
                self.assertTrue(all(density > 0 for density in densities))
                self.assertTrue(math.isclose(sum(densities), 384 / 1_000_000, rel_tol=1e-12))


class TestCraterRadiusBins(unittest.TestCase):
    def test_automatic_bins_place_the_largest_craters_first(self):
        self.assertEqual(compute_radius_bins([0.5, 2.0], 0.5), [[1.5, 2.0], [1.0, 1.5], [0.5, 1.0]])


class TestCraterDistributionConf(unittest.TestCase):
    def test_empty_lists_are_generated_from_the_power_law(self):
        conf = CraterDistributionConf(
            x_size=10.0, y_size=10.0, radius_range=[0.5, 1.5], radius_step=0.5, dist_coef=100.0, dist_exp=-2.0, seed=1
        )

        self.assertEqual(conf.radius, [[1.0, 1.5], [0.5, 1.0]])
        self.assertEqual(conf.densities, compute_bin_densities(conf.radius, 100.0, -2.0))

    def test_explicit_lists_must_have_the_same_length(self):
        with self.assertRaisesRegex(AssertionError, "same length"):
            CraterDistributionConf(x_size=10.0, y_size=10.0, radius=[[1.0, 2.0], [2.0, 3.0]], densities=[0.1], seed=1)


if __name__ == "__main__":
    unittest.main()
