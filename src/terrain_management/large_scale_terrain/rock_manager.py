__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import List, Tuple, Dict, Union
import numpy as np
import dataclasses
import threading
import logging
import time
import os

from semantics.schema.editor import PrimSemanticData
import omni

from src.terrain_management.large_scale_terrain.pxr_utils import (
    add_collider,
    load_material,
    bind_material,
    delete_prim,
    set_xform_ops,
)
from src.terrain_management.large_scale_terrain.rock_distribution import RockSamplerConf, RockSampler
from src.terrain_management.large_scale_terrain.utils import BoundingBox, RockBlockData, ScopedTimer
from src.terrain_management.large_scale_terrain.rock_database import RockDB, RockDBConf

from pxr import UsdGeom, Gf, Usd, Vt, Sdf

logger = logging.getLogger(__name__)
logging.basicConfig(format="%(asctime)s %(message)s", datefmt="%m/%d/%Y %I:%M:%S %p")


class Instancer:
    """
    Point instancer object with extra capabilities regarding colliders, texturing and semantic labeling.
    """

    def __init__(
        self,
        instancer_path: str,
        assets_path: str,
        seed: int,
        add_colliders: bool = False,
        collider_mode: str = None,
        semantic_label: str = None,
        texture_name: str = None,
        texture_path: str = None,
    ):
        """
        Args:
            instancer_path (str): The path to the instancer.
            assets_path (str): The path to the assets folder.
            seed (int): The seed for the random number generator.
            add_colliders (bool): flag to indicate if colliders should be added.
            collider_mode (str): mode of the collider. (None to use default mode)
            semantic_label (str): semantic label of the rocks. (None if no label is to be used)
            texture_name (str): name of the texture. (None if no texture is to be used)
            texture_path (str): path to the texture. (None if no texture is to be used)
        """

        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.assets_path = assets_path
        self.add_colliders = add_colliders
        self.apply_material = False
        self.add_semantic_label = True if semantic_label is not None else False
        self.collider_mode = collider_mode
        self.semantic_label = semantic_label
        self.texture_name = texture_name
        self.texture_path = texture_path
        self.prototypes = []
        self.get_asset_list()
        self.load_material()
        self.create_instancer()
        self.rng = np.random.default_rng(seed=seed)

    def get_asset_list(self):
        """
        Get the list of assets in the assets folder.
        """

        self.file_paths = [
            os.path.join(self.assets_path, file)
            for file in os.listdir(self.assets_path)
            if (file.endswith(".usd") or file.endswith(".usda") or file.endswith(".usdz"))
        ]

    def load_material(self):
        """
        Load the material.

        Warnings:
            - If the texture name is not provided, a warning is raised.
            - If the texture path is not provided, a warning is raised.
        """
        self.apply_material = False
        if (self.texture_name is not None) and (self.texture_path is not None):
            self.material_path = load_material(self.texture_name, self.texture_path)
            self.apply_material = True
        elif (self.texture_name is not None) and (self.texture_path is None):
            logger.warn("Texture path not provided. Material will not be loaded.")
        elif (self.texture_name is None) and (self.texture_path is not None):
            logger.warn("Texture name not provided. Material will not be loaded.")

    def apply_semantic_label(self, prim_path: Usd.Prim):
        prim_sd = PrimSemanticData(self.stage.GetPrimAtPath(prim_path))
        prim_sd.add_entry("class", self.semantic_label)

    def create_instancer(self):
        """
        Create the instancer.
        """

        self.instancer_path = omni.usd.get_stage_next_free_path(self.stage, self.instancer_path, False)
        self.instancer = UsdGeom.PointInstancer.Define(self.stage, self.instancer_path)
        self.instancer_prim = self.instancer.GetPrim()

        prim_paths = []
        for i, file_path in enumerate(self.file_paths):
            prim = self.stage.DefinePrim(os.path.join(self.instancer_path, "asset_" + str(i)), "Xform")
            prim.GetReferences().AddReference(file_path)
            if self.add_colliders:
                add_collider(self.stage, prim.GetPath(), mode=self.collider_mode)
            if self.apply_material:
                bind_material(self.stage, self.material_path, prim.GetPath())
            if self.add_semantic_label:
                self.apply_semantic_label(prim.GetPath())

            prim_paths.append(prim.GetPath())

        self.instancer.GetPrototypesRel().SetTargets(prim_paths)

        self.set_instancer_parameters(
            np.array([[0, 0, 0]]), np.array([[0, 0, 0, 1]]), np.array([[1, 1, 1]]), np.array([0])
        )

    def set_instancer_parameters(
        self, positions: np.ndarray, orientations: np.ndarray, scales: np.ndarray, ids: np.ndarray
    ) -> None:
        """
        Set the instancer parameters.

        Args:
            positions (np.ndarray): The positions of the instances.
            orientations (np.ndarray): The orientations of the instances.
            scales (np.ndarray): The scales of the instances.
            ids (np.ndarray): The ids of the instances.
        """

        self.instancer.GetPositionsAttr().Set(positions)
        self.instancer.GetOrientationsAttr().Set(orientations)
        self.instancer.GetScalesAttr().Set(scales)
        self.instancer.GetProtoIndicesAttr().Set(ids)

        try:
            self.update_extent()
        except Exception as e:
            logger.warn("Error updating extent", e)

    def update_extent(self) -> None:
        """
        Updates the extent of an instancer.
        """

        # Compute the extent of the objetcs.
        extent = self.instancer.ComputeExtentAtTime(Usd.TimeCode(0), Usd.TimeCode(0))
        # Applies the extent to the instancer.
        self.instancer.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(extent[0]), Gf.Vec3f(extent[1])]))

    def get_num_prototypes(self) -> int:
        """
        Get the number of prototypes.

        Returns:
            int: The number of prototypes.
        """

        return len(self.file_paths)


class CustomInstancer:
    """
    StandaloneInstancer class. It is used to create an instancer that allows to spawn multiple instances of the same assets.
    We defined our own instancer instead of using a Usd.Geom.PointInstancer as the latter does not support semantic information properly.
    It may be fixed in the next release of Omniverse?"""

    def __init__(
        self,
        instancer_path: str,
        assets_path: str,
        seed: int,
        add_colliders: bool = False,
        collider_mode: str = None,
        semantic_label: str = None,
        texture_name: str = None,
        texture_path: str = None,
    ):
        """
        Args:
            instancer_path (str): The path of the instancer.
            asset_list (list): The list of assets that can be spawned by the instancer.
            seed (int, optional): The seed used to generate the random numbers. Defaults to 0.
        """

        self.instance_paths = []
        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.assets_path = assets_path
        self.add_colliders = add_colliders
        self.apply_material = False
        self.add_semantic_label = True if semantic_label is not None else False
        self.collider_mode = collider_mode
        self.semantic_label = semantic_label
        self.texture_name = texture_name
        self.texture_path = texture_path
        self.stage = omni.usd.get_context().get_stage()
        self.get_asset_list()
        self.load_material()
        self.create_instancer()
        self.rng = np.random.default_rng(seed=seed)

    def get_asset_list(self):
        """
        Get the list of assets in the assets folder.
        """

        self.file_paths = [
            os.path.join(self.assets_path, file)
            for file in os.listdir(self.assets_path)
            if (file.endswith(".usd") or file.endswith(".usda") or file.endswith(".usdz"))
        ]

    def load_material(self):
        """
        Load the material.

        Warnings:
            - If the texture name is not provided, a warning is raised.
            - If the texture path is not provided, a warning is raised.
        """
        self.apply_material = False
        if (self.texture_name is not None) and (self.texture_path is not None):
            self.material_path = load_material(self.texture_name, self.texture_path)
            self.apply_material = True
        elif (self.texture_name is not None) and (self.texture_path is None):
            logger.warn("Texture path not provided. Material will not be loaded.")
        elif (self.texture_name is None) and (self.texture_path is not None):
            logger.warn("Texture name not provided. Material will not be loaded.")

    def apply_semantic_label(self, prim_path: Usd.Prim):
        prim_sd = PrimSemanticData(self.stage.GetPrimAtPath(prim_path))
        prim_sd.add_entry("class", self.semantic_label)

    def create_instancer(self):
        """
        Create the instancer.
        """

        self.instancer_path = omni.usd.get_stage_next_free_path(self.stage, self.instancer_path, False)
        instancer_prim = self.stage.DefinePrim(self.instancer_path, "Xform")
        instancer_prim = UsdGeom.Xformable(instancer_prim)

        self.prim_paths = []
        for i, file_path in enumerate(self.file_paths):
            prim = self.stage.DefinePrim(os.path.join(self.instancer_path, "asset_" + str(i)), "Xform")
            prim.GetReferences().AddReference(file_path)
            if self.add_colliders:
                add_collider(self.stage, prim.GetPath(), mode=self.collider_mode)
            if self.apply_material:
                bind_material(self.stage, self.material_path, prim.GetPath())
            if self.add_semantic_label:
                self.apply_semantic_label(prim.GetPath())
            prim.SetInstanceable(True)
            prim.GetAttribute("visibility").Set("invisible")

            self.prim_paths.append(prim.GetPath())

        self.set_instancer_parameters(
            np.array([[0, 0, 0]]), np.array([[0, 0, 0, 1]]), np.array([[1, 1, 1]]), np.array([0])
        )

    def set_instancer_parameters(
        self, position: np.ndarray, orientation: np.ndarray, scale: np.ndarray, ids: np.ndarray
    ) -> None:
        """
        Set the instancer's parameters. It sets the position, orientation, scale, and semantic class of the instances.

        Args:
            position (np.ndarray): The position of the instances.
            orientation (np.ndarray): The orientation of the instances.
            scale (np.ndarray): The scale of the instances.
            ids (np.ndarray): The ids of the instances.
        """

        self.destroy()
        self.num = position.shape[0]
        self.position = position
        self.orientation = orientation
        self.scale = scale
        self.ids = ids
        self.update()

    def update(self) -> None:
        """
        Update the instancer. It creates the instances and set their parameters.
        It also sets the semantic class of the instances if it is not None.
        """

        self.instance_paths = []
        for i, id in enumerate(self.ids):
            source_prim_path = self.file_paths[id]
            prefix = os.path.join(self.instancer_path, f"instance_{i}")
            self.instance_paths.append(prefix)
            prim = self.stage.DefinePrim(prefix, "Xform")
            prim.GetReferences().AddReference(source_prim_path)

            if self.add_colliders:
                add_collider(self.stage, prim.GetPath(), mode=self.collider_mode)
            if self.apply_material:
                bind_material(self.stage, self.material_path, prim.GetPath())
            if self.add_semantic_label:
                self.apply_semantic_label(prim.GetPath())
            prim.SetInstanceable(True)
            set_xform_ops(
                prim,
                Gf.Vec3d(float(self.position[i, 0]), float(self.position[i, 1]), float(self.position[i, 2])),
                Gf.Quatd(
                    float(self.orientation[i, 0]),
                    (float(self.orientation[i, 1]), float(self.orientation[i, 2]), float(self.orientation[i, 3])),
                ),
                Gf.Vec3d(float(self.scale[i, 0]), float(self.scale[i, 1]), float(self.scale[i, 2])),
            )

    def destroy(self):
        """
        Destroy the instancer."""

        for instance_path in self.instance_paths:
            delete_prim(self.stage, instance_path)
        self.num = None
        self.position = None
        self.orientation = None
        self.scale = None
        self.ids = None
        self.semantic_classes = None

    def get_num_prototypes(self) -> int:
        """
        Get the number of prototypes.

        Returns:
            int: The number of prototypes.
        """

        return len(self.file_paths)


class ChangeBlock:
    """
    Point instancer object with extra capabilities regarding colliders, texturing and semantic labeling.
    """

    def __init__(
        self,
        instancer_path: str,
        assets_path: str,
        seed: int,
        add_colliders: bool = False,
        collider_mode: str = None,
        semantic_label: str = None,
        texture_name: str = None,
        texture_path: str = None,
    ):
        """
        Args:
            instancer_path (str): The path to the instancer.
            assets_path (str): The path to the assets folder.
            seed (int): The seed for the random number generator.
            add_colliders (bool): flag to indicate if colliders should be added.
            collider_mode (str): mode of the collider. (None to use default mode)
            semantic_label (str): semantic label of the rocks. (None if no label is to be used)
            texture_name (str): name of the texture. (None if no texture is to be used)
            texture_path (str): path to the texture. (None if no texture is to be used)
        """

        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.assets_path = assets_path
        self.add_colliders = add_colliders
        self.apply_material = False
        self.add_semantic_label = True if semantic_label is not None else False
        self.collider_mode = collider_mode
        self.semantic_label = semantic_label
        self.texture_name = texture_name
        self.texture_path = texture_path
        self.prototypes = []
        self.get_asset_list()
        self.load_material()
        self.create_instancer()
        self.rng = np.random.default_rng(seed=seed)

    def get_asset_list(self):
        """
        Get the list of assets in the assets folder.
        """

        self.file_paths = [
            os.path.join(self.assets_path, file)
            for file in os.listdir(self.assets_path)
            if (file.endswith(".usd") or file.endswith(".usda") or file.endswith(".usdz"))
        ]

    def load_material(self):
        """
        Load the material.

        Warnings:
            - If the texture name is not provided, a warning is raised.
            - If the texture path is not provided, a warning is raised.
        """
        self.apply_material = False
        if (self.texture_name is not None) and (self.texture_path is not None):
            self.material_path = load_material(self.texture_name, self.texture_path)
            self.apply_material = True
        elif (self.texture_name is not None) and (self.texture_path is None):
            logger.warn("Texture path not provided. Material will not be loaded.")
        elif (self.texture_name is None) and (self.texture_path is not None):
            logger.warn("Texture name not provided. Material will not be loaded.")

    def apply_semantic_label(self, prim_path: Usd.Prim):
        prim_sd = PrimSemanticData(self.stage.GetPrimAtPath(prim_path))
        prim_sd.add_entry("class", self.semantic_label)

    def create_instancer(self):
        """
        Create the instancer.
        """

        self.prim_paths = []
        for i, file_path in enumerate(self.file_paths):
            prim = self.stage.DefinePrim(os.path.join(self.instancer_path, "asset_" + str(i)), "Xform")
            prim.GetReferences().AddReference(file_path)
            if self.add_colliders:
                add_collider(self.stage, prim.GetPath(), mode=self.collider_mode)
            if self.apply_material:
                bind_material(self.stage, self.material_path, prim.GetPath())
            if self.add_semantic_label:
                self.apply_semantic_label(prim.GetPath())

            self.prim_paths.append(prim.GetPath())

        self.set_instancer_parameters(
            np.array([[0.0, 0.0, 0.0]]), np.array([[0.0, 0.0, 0.0, 1.0]]), np.array([[1.0, 1.0, 1.0]]), np.array([0])
        )

    def set_instancer_parameters(
        self, positions: np.ndarray, quats: np.ndarray, scales: np.ndarray, ids: np.ndarray
    ) -> None:
        """
        Set the instancer parameters.

        Args:
            positions (np.ndarray): The positions of the instances.
            quats (np.ndarray): The quaternions of the instances.
            scales (np.ndarray): The scales of the instances.
            ids (np.ndarray): The ids of the instances.
        """

        with Sdf.ChangeBlock():
            layer = self.stage.GetRootLayer()
            for i, id in enumerate(ids):
                prim_path = self.instancer_path + "/asset_2_" + str(i)
                prim_spec = Sdf.CreatePrimInLayer(layer, prim_path)
                Sdf.CopySpec(prim_spec.layer, Sdf.Path(self.prim_paths[id]), prim_spec.layer, Sdf.Path(prim_path))

                position_spec = prim_spec.GetAttributeAtPath(prim_path + ".xformOp:translate")
                if position_spec is None:
                    position_spec = Sdf.AttributeSpec(prim_spec, "xformOp:translate", Sdf.ValueTypeNames.Float3)
                position_spec.default = Gf.Vec3f(positions[i, 0], positions[i, 1], positions[i, 2])

                orient_spec = prim_spec.GetAttributeAtPath(prim_path + ".xformOp:orient")
                if orient_spec is None:
                    orient_spec = Sdf.AttributeSpec(prim_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatf)
                orient_spec.default = Gf.Quatf(
                    float(quats[i, 3]), (float(quats[i, 0]), float(quats[i, 1]), float(quats[i, 2]))
                )

                scale_spec = prim_spec.GetAttributeAtPath(prim_path + ".xformOp:scale")
                if scale_spec is None:
                    scale_spec = Sdf.AttributeSpec(prim_spec, "xformOp:scale", Sdf.ValueTypeNames.Float3)
                scale_spec.default = Gf.Vec3f(scales[i, 0], scales[i, 1], scales[i, 2])

                xform_order_spec = prim_spec.GetAttributeAtPath(prim_path + ".xformOpOrder")
                if xform_order_spec is None:
                    xform_order_spec = Sdf.AttributeSpec(prim_spec, "xformOpOrder", Sdf.ValueTypeNames.TokenArray)
                xform_order_spec.default = Vt.TokenArray(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

    def get_num_prototypes(self) -> int:
        """
        Get the number of prototypes.

        Returns:
            int: The number of prototypes.
        """

        return len(self.file_paths)


@dataclasses.dataclass
class RockGeneratorConf:
    """
    Args:
        rock_db_cfg (RockDBConf): The configuration for the rock database.
        rock_sampler_cfg (RockSamplerConf): The configuration for the rock sampler.
        rock_assets_folder (str): The path to the rock assets folder.
        instancer_name (str): The name of the instancer.
        seed (int): The seed for the random number generator.
        block_span (int): The number of blocks to sample around the position.
        block_size (int): The size of the blocks in meters.
        has_colliders (bool): flag to indicate if the blocks have colliders.
        semantic_label (str): semantic label of the rocks. (None if no label is to be used)
        texture_name (str): name of the texture. (None if no texture is to be used)
        texture_path (str): path to the texture. (None if no texture is to be used)
    """

    rock_db_cfg: RockDBConf = dataclasses.field(default_factory=dict)
    rock_sampler_cfg: RockSamplerConf = dataclasses.field(default_factory=dict)
    rock_assets_folder: str = dataclasses.field(default_factory=list)
    instancer_name: str = dataclasses.field(default_factory=str)
    seed: int = dataclasses.field(default_factory=int)
    block_span: int = dataclasses.field(default_factory=int)
    block_size: int = dataclasses.field(default_factory=int)
    add_colliders: bool = dataclasses.field(default_factory=bool)
    collider_mode: str = dataclasses.field(default_factory=str)
    semantic_label: str = dataclasses.field(default_factory=str)
    texture_name: str = dataclasses.field(default_factory=str)
    texture_path: str = dataclasses.field(default_factory=str)

    def __post_init__(self):
        assert self.instancer_name != "", "Instancer name cannot be empty"
        assert self.block_span >= 0, "Block span must be greater or equal to 0"
        assert os.path.exists(self.rock_assets_folder), "Rock assets folder not found"

        if self.collider_mode == "":
            self.collider_mode = None
        if self.semantic_label == "":
            self.semantic_label = None
        if self.texture_name == "":
            self.texture_name = None
        if self.texture_path == "":
            self.texture_path = None

        self.rock_db_cfg = RockDBConf(**self.rock_db_cfg)
        self.rock_sampler_cfg = RockSamplerConf(**self.rock_sampler_cfg)


class RockGenerator:
    """
    The rock generator is responsible for generating rocks on the terrain. It samples the rocks
    around a given position and updates the instancer with the new data.
    """

    def __init__(
        self, settings: RockGeneratorConf, sampling_func: callable, is_map_done: callable, instancer_path: str
    ):
        """
        Args:
            settings (RockGeneratorConf): The settings for the rock generator.
            sampling_func (callable): The function to sample the rocks.
            is_map_done (callable): The function to check if the map is done.
            instancer_path (str): The path to the instancer.
        """

        self.settings = settings
        self.sampling_func = sampling_func
        self.is_map_done = is_map_done
        self.instancer_path = instancer_path
        self.is_sampling = False

    def build(self) -> None:
        """
        Builds the rock generator. It initializes the rock database, the rock sampler, and the
        rock instancer.
        """

        self.rock_db = RockDB(self.settings.rock_db_cfg)
        if self.settings.rock_sampler_cfg.seed is None:
            self.settings.rock_sampler_cfg.seed = self.settings.seed + 1

        if self.settings.semantic_label is not None:
            self.rock_instancer = CustomInstancer(
                os.path.join(self.instancer_path, self.settings.instancer_name),
                self.settings.rock_assets_folder,
                self.settings.seed,
                self.settings.add_colliders,
                self.settings.collider_mode,
                self.settings.semantic_label,
                self.settings.texture_name,
                self.settings.texture_path,
            )
        else:
            self.rock_instancer = Instancer(
                os.path.join(self.instancer_path, self.settings.instancer_name),
                self.settings.rock_assets_folder,
                self.settings.seed,
                self.settings.add_colliders,
                self.settings.collider_mode,
                self.settings.semantic_label,
                self.settings.texture_name,
                self.settings.texture_path,
            )

        self.rock_sampler = RockSampler(
            self.settings.rock_sampler_cfg,
            self.rock_db,
            map_sampling_func=self.sampling_func,
            num_objects=self.rock_instancer.get_num_prototypes(),
        )

    def cast_coordinates_to_block_space(self, coordinates: Tuple[float, float]) -> Tuple[int, int]:
        """
        Casts the given coordinates to the block space. The block space is the space
        where the blocks are defined. The block space is defined by the block size and
        the resolution of the high resolution DEM.

        The coordinates are still expressed in meters, but they can only be an increment of
        the block size (in meters).

        Args:
            coordinates (Tuple[float, float]): The coordinates to cast to the block space.

        Returns:
            Tuple[int, int]: The coordinates in the block space.
        """

        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def define_region(self, coordinates: Tuple[float, float]) -> BoundingBox:
        """
        Defines the region to sample the rocks. The region is defined by the coordinates
        and the block span. The block span is the number of blocks to sample around the
        coordinates.

        The region is defined by the following formula:
            x_low = coordinates[0] - (block_span + 1) * block_size
            x_high = coordinates[0] + (block_span + 2) * block_size
            y_low = coordinates[1] - (block_span + 1) * block_size
            y_high = coordinates[1] + (block_span + 2) * block_size

        Note the +1 on the minimum and +2 on the maximum. This is done to have a buffer of
        blocks around the sampling region.

        Args:
            coordinates (Tuple[float, float]): The coordinates to define the region around.

        Returns:
            BoundingBox: The bounding box of the region.
        """

        x_low = coordinates[0] - (self.settings.block_span) * self.settings.block_size
        x_high = coordinates[0] + (self.settings.block_span + 1) * self.settings.block_size
        y_low = coordinates[1] - (self.settings.block_span) * self.settings.block_size
        y_high = coordinates[1] + (self.settings.block_span + 1) * self.settings.block_size

        return BoundingBox(x_min=x_low, x_max=x_high, y_min=y_low, y_max=y_high)

    def aggregate_block_data(
        self, blocks: List[RockBlockData]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Aggregates the block data into numpy arrays. The block data is a list of dictionaries
        where each dictionary contains the following keys:
            - position: The position of the block.
            - orientation: The orientation of the block.
            - scale: The scale of the block.
            - id: The id of the block.

        Args:
            blocks (List[RockBlockData]): The list of block data to aggregate.

        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: The position, orientation,
            scale, and ids as numpy arrays
        """

        position = np.vstack([block.coordinates for block in blocks])
        orientation = np.vstack([block.quaternion for block in blocks])
        scale = np.vstack([block.scale for block in blocks])
        ids = np.hstack([block.ids for block in blocks])

        return position, orientation, scale, ids

    def update_instancer(self, region: BoundingBox) -> None:
        """
        Updates the instancer with the new block data. The function gets the blocks within
        the region and aggregates the block data. It then sets the instancer's parameters
        with the new data.

        Args:
            region (BoundingBox): The region to sample the rocks.
        """

        blocks, _, _ = self.rock_db.get_blocks_within_region(region)
        position, orientation, scale, ids = self.aggregate_block_data(blocks)
        self.rock_instancer.set_instancer_parameters(position, orientation, scale, ids)

    def sample(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position. The function casts the position to the block
        space and defines the region around the position. It then samples the rocks within the
        region and updates the instancer with the new data.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
            map_coordinates (Tuple[float, float]): The coordinates in the map space.
        """

        while not self.is_map_done():
            time.sleep(0.1)

        self.is_sampling = True
        coordinates = self.cast_coordinates_to_block_space(global_position)
        region = self.define_region(coordinates)
        self.rock_sampler.sample_rocks_by_region(region, global_position)
        self.update_instancer(region)
        self.is_sampling = False

    def _sample_threaded(self, position: Tuple[float, float]) -> None:
        """
        Internal function DO NOT CALL.

        This function is used to sample the rocks in a separate thread. It waits for potential previous
        calls to finish before starting a new one.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
        """

        while self.is_sampling:
            time.sleep(0.1)

        self.sample(position)

    def sample_threaded(self, position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position in a separate thread. The function waits for potential
        previous calls to finish before starting a new one.

        Args:
            position (Tuple[float, float]): The position to sample the rocks.
        """

        thread = threading.Thread(target=self._sample_threaded, args=(position,))
        thread.start()


@dataclasses.dataclass
class RockManagerConf:
    """
    Args:
        rock_gen_cfgs (List[RockGeneratorConf]): The configuration for the rock generators.
        instancers_path (str): The path to the instancers.
        seed (int): The seed for the random number generator.
    """

    rock_gen_cfgs: List[RockGeneratorConf] = dataclasses.field(default_factory=list)
    instancers_path: str = dataclasses.field(default_factory=str)
    seed: int = dataclasses.field(default_factory=int)
    block_size: int = dataclasses.field(default_factory=int)
    rock_dbs_cfg: RockDBConf = dataclasses.field(default_factory=dict)
    profiling: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        for cfg in self.rock_gen_cfgs:
            cfg["rock_db_cfg"] = self.rock_dbs_cfg
            cfg["block_size"] = self.block_size
        self.rock_gen_cfgs = [RockGeneratorConf(**cfg) for cfg in self.rock_gen_cfgs]


class RockManager:
    """
    The rock manager is responsible for managing the rocks on the terrain. It manages multiple
    rock generators and samples the rocks around a given position.
    """

    def __init__(self, settings: RockManagerConf, sampling_func: callable, is_map_done: callable) -> None:
        """
        Args:
            settings (RockManagerConf): The settings for the rock manager.
            sampling_func (callable): The function to sample the rocks height and normals.
            is_map_done (callable): The function to check if the map is done.
        """

        self.settings = settings
        self.sampling_func = sampling_func
        self.is_map_done = is_map_done
        self.last_update = None

    def build(self) -> None:
        """
        Builds the rock manager. It initializes the rock generators.
        """

        self.rock_generators: List[RockGenerator] = []

        for i, rock_gen_cfg in enumerate(self.settings.rock_gen_cfgs):
            if rock_gen_cfg.seed is None:
                rock_gen_cfg.seed = self.settings.seed + i * 4
            rock_generator = RockGenerator(
                rock_gen_cfg, self.sampling_func, self.is_map_done, self.settings.instancers_path
            )
            rock_generator.build()
            self.rock_generators.append(rock_generator)

    def cast_coordinates_to_block_space(self, coordinates: Tuple[float, float]) -> Tuple[int, int]:
        """
        Casts the given coordinates to the block space. The block space is the space
        where the blocks are defined. The block space is defined by the block size and
        the resolution of the high resolution DEM.

        The coordinates are still expressed in meters, but they can only be an increment of
        the block size (in meters).

        Args:
            coordinates (Tuple[float, float]): The coordinates to cast to the block space.

        Returns:
            Tuple[int, int]: The coordinates in the block space.
        """

        x, y = coordinates
        x_block = int(x // self.settings.block_size) * self.settings.block_size
        y_block = int(y // self.settings.block_size) * self.settings.block_size
        return (x_block, y_block)

    def check_if_update_needed(self, position: Tuple[float, float]) -> bool:
        """
        Checks if an update is needed. The function checks if the position is different from the
        last update in the block space coordinates.

        Args:
            position (Tuple[float, float]): The position to check. (block space coordinates)
        """
        if self.last_update is None:
            return True
        else:
            x, y = position
            x_last, y_last = self.last_update
            return (x != x_last) or (y != y_last)

    def sample(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position. The function samples the rocks for each rock
        generator.

        Args:
            global_position (Tuple[float, float]): The position to sample the rocks.
            map_coordinates (Tuple[float, float]): The coordinates in the map space.
        """
        position = self.cast_coordinates_to_block_space(global_position)

        if self.check_if_update_needed(position):
            self.last_update = position
            for rock_generator in self.rock_generators:
                with ScopedTimer("RockManager sample", active=self.settings.profiling):
                    rock_generator.sample(position)

    def sample_threaded(self, global_position: Tuple[float, float]) -> None:
        """
        Samples the rocks at the given position in a separate thread. The function samples the rocks
        for each rock generator in a separate thread.

        Args:
            global_position (Tuple[float, float]): The position to sample the rocks.
        """

        position = self.cast_coordinates_to_block_space(global_position)
        if self.check_if_update_needed(position):
            for rock_generator in self.rock_generators:
                rock_generator.sample_threaded(position)
