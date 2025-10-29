__author__ = "Antoine Richard, Junnosuke Kahamora"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import numpy as np
import omni
import os
from isaacsim.core.utils.prims import delete_prim
from isaacsim.core.utils.semantics import add_update_semantics
from pxr import UsdGeom, Gf
from WorldBuilders.pxr_utils import createObject, createXform


def castVec3d(data: np.ndarray):
    return Gf.Vec3d(data[0], data[1], data[2])


def castRot(data: np.ndarray):
    return Gf.Quatd(data[3], data[0], data[1], data[2])


class CustomInstancer:
    """
    StandaloneInstancer class. It is used to create an instancer that allows to spawn multiple instances of the same assets.
    We defined our own instancer instead of using a Usd.Geom.PointInstancer as the latter does not support semantic information properly.
    It may be fixed in the next release of Omniverse?"""

    def __init__(
        self,
        instancer_path: str,
        asset_list: list,
        semantic_class: str = None,
        seed: int = 0,
    ):
        """
        Args:
            instancer_path (str): The path of the instancer.
            asset_list (list): The list of assets that can be spawned by the instancer.
            seed (int, optional): The seed used to generate the random numbers. Defaults to 0.
        """

        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        instancer_prim, _ = createXform(self.stage, instancer_path)
        instancer_prim = UsdGeom.Xformable(instancer_prim)
        self.prototypes = asset_list
        self.flag = False
        self.makeCache()
        self.instance_paths = []
        self.semantic_class = semantic_class
        self.rng = np.random.default_rng(seed=seed)

    def setInstanceParameter(self, position: np.ndarray, orientation: np.ndarray, scale: np.ndarray) -> None:
        """
        Set the instancer's parameters. It sets the position, orientation, scale, and semantic class of the instances.

        Args:
            position (np.ndarray): The position of the instances.
            orientation (np.ndarray): The orientation of the instances.
            scale (np.ndarray): The scale of the instances.
            semantic_class (str, optional): The semantic class of the instances. Defaults to None.
        """

        self.destroy()
        self.num = position.shape[0]
        self.position = position
        self.orientation = orientation
        self.scale = scale
        self.ids = self.rng.integers(0, len(self.prototypes), self.num)
        self.semantic_classes = [self.semantic_class] * self.num
        self.update()

    def makeCache(self) -> None:
        """
        Create a cache for the instancer. The cache is a bank of all the assets that can be spawned by the instancer.
        These assets are hidden from the scene. The idea being that when the instancer spawns an instance,
        it will use the cached asset instead of loading it from the disk."""

        self.cache_path = os.path.join(self.instancer_path, "cache")
        self.cache_prim = createXform(self.stage, self.cache_path)
        for asset in self.prototypes:
            prefix = os.path.join(self.cache_path, "cached_obj")
            prim, _ = createObject(prefix, self.stage, asset, is_instance=True)
            prim.GetAttribute("visibility").Set("invisible")

    def update(self):
        """
        Update the instancer. It creates the instances and set their parameters.
        It also sets the semantic class of the instances if it is not None."""

        self.instance_paths = []
        for i, id in enumerate(self.ids):
            source_prim_path = self.prototypes[id]
            prefix = os.path.join(self.instancer_path, f"instance_{i}")
            self.instance_paths.append(prefix)
            prim, _ = createObject(
                prefix,
                self.stage,
                source_prim_path,
                castVec3d(self.position[i]),
                castRot(self.orientation[i]),
                castVec3d(self.scale[i]),
                True,
            )
            if self.semantic_classes[i] is not None:
                add_update_semantics(prim, self.semantic_classes[i])
        self.flag = True

    def destroy(self):
        """
        Destroy the instancer."""

        for instance_path in self.instance_paths:
            delete_prim(instance_path)
        self.num = None
        self.position = None
        self.orientation = None
        self.scale = None
        self.ids = None
        self.semantic_classes = None
