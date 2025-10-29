__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

# System imports
from scipy.spatial.transform import Rotation as SSTR
import numpy as np

# Once the sim is started load isaac libs (including ROS)
from omni.isaac.core import World
from pxr import Gf
import omni

# Custom libs
from src.environments.lunalab import LunalabController

from src.configurations.procedural_terrain_confs import TerrainManagerConf
from src.configurations.auto_labeling_confs import CameraConf
from src.configurations.rendering_confs import FlaresConf
from src.configurations.environments import LunalabConf
from src.labeling.auto_label import AutonomousLabeling
from assets import get_assets_path

from WorldBuilders.pxr_utils import setDefaultOps, addDefaultOps
from WorldBuilders.Types import Position_T, Orientation_T, UserRequest_T
from WorldBuilders.Types import UniformSampler_T, ImageClipper_T
from WorldBuilders.Types import Image_T, RollPitchYaw_T
from WorldBuilders.Mixer import RequestMixer

from pxr import UsdGeom, Gf


class SDG_Lunalab(LunalabController):
    def __init__(
        self,
        lunalab_settings: LunalabConf = None,
        rocks_settings: dict = None,
        flares_settings: FlaresConf = None,
        terrain_manager: TerrainManagerConf = None,
        camera_settings: CameraConf = None,
        **kwargs,
    ) -> None:
        super().__init__(
            lunalab_settings=lunalab_settings,
            rocks_settings=rocks_settings,
            flares_settings=flares_settings,
            terrain_manager=terrain_manager,
            **kwargs,
        )
        self.camera_settings = camera_settings
        self.terrain_settings = terrain_manager
        self.counter = 0
        self.rng = np.random.default_rng(seed=terrain_manager.moon_yard.crater_distribution.seed)

    def load(self) -> None:
        self.createCamera()
        super().load()

    def createCamera(self):
        """
        Creates the camera."""
        # The prim that contains the camera prim itself.
        self._camera_prim = self.stage.DefinePrim(self.scene_name + "/Camera", "Xform")
        # The camera path to the camera prim
        self._camera_path = self.scene_name + "/Camera/camera_annotations"
        # Creates a camera
        self._camera: UsdGeom.Camera = UsdGeom.Camera.Define(self.stage, self._camera_path)
        # Rigs the camera using the settings provided by the user
        self._camera.CreateFocalLengthAttr().Set(self.camera_settings.focal_length)
        self._camera.CreateFocusDistanceAttr().Set(self.camera_settings.focus_distance)
        self._camera.CreateHorizontalApertureAttr().Set(self.camera_settings.horizontal_aperture)
        self._camera.CreateVerticalApertureAttr().Set(self.camera_settings.vertical_aperture)
        self._camera.CreateFStopAttr().Set(self.camera_settings.fstop)
        self._camera.CreateClippingRangeAttr().Set(
            Gf.Vec2f(
                self.camera_settings.clipping_range[0],
                self.camera_settings.clipping_range[1],
            )
        )
        # Offsets the camera position from the ground by 20cm (0.2m)
        # Rotates the camera such that it looks forward
        addDefaultOps(UsdGeom.Xformable(self._camera.GetPrim()))
        setDefaultOps(
            UsdGeom.Xformable(self._camera.GetPrim()),
            (0.0, 0.0, 0.2),
            (0.5, -0.5, -0.5, 0.5),
            (1.0, 1.0, 1.0),
        )
        addDefaultOps(UsdGeom.Xformable(self._camera_prim))
        setDefaultOps(
            UsdGeom.Xformable(self._camera_prim),
            (0.0, 0.0, 0.0),
            (0, 0, 0, 1),
            (1.0, 1.0, 1.0),
        )

    def createCameraSampler(self) -> None:
        """
        Creates the sampler for a camera."""

        H, W = self.dem.shape

        # Generates the requests to be sent to the procedural camera placement.
        # Positiion based on the mask
        xy_mask = Image_T(
            data=self.mask,
            mpp_resolution=self.terrain_settings.resolution,
            output_space=2,
        )
        xy_sampler = UniformSampler_T(
            min=(0.75, self.terrain_settings.sim_length - 0.75),
            max=(0.75, self.terrain_settings.sim_width - 0.75),
            randomization_space=2,
            seed=42,
        )
        req_pos_xy = UserRequest_T(p_type=Position_T(), sampler=xy_sampler, layer=xy_mask, axes=["x", "y"])
        # Random yaw
        rpy_layer = RollPitchYaw_T(rmax=0, rmin=0, pmax=0, pmin=0, ymax=np.pi * 2, ymin=0)
        rpy_sampler = UniformSampler_T(randomization_space=3, seed=42)
        req_ori = UserRequest_T(
            p_type=Orientation_T(),
            sampler=rpy_sampler,
            layer=rpy_layer,
            axes=["x", "y", "z", "w"],
        )
        # DEM clipper
        image_layer = Image_T(output_space=1)
        image_clipper = ImageClipper_T(
            randomization_space=1,
            resolution=(H, W),
            mpp_resolution=self.terrain_settings.resolution,
            data=self.dem,
        )
        req_pos_z = UserRequest_T(p_type=Position_T(), sampler=image_clipper, layer=image_layer, axes=["z"])
        requests = [req_pos_xy, req_pos_z, req_ori]
        self.mixer_camera = RequestMixer(requests)

    def randomizeProjector(self):
        x = self.rng.uniform(0.5, self.terrain_settings.sim_width - 0.5)
        z = self.rng.uniform(0.3, 1.5)
        y = 0
        theta = self.rng.uniform(0, 20) - 10
        phi = self.rng.uniform(0, 20) - 10

        R = SSTR.from_euler("xyz", (phi, 0, theta), degrees=True)
        quat = R.as_quat()
        setDefaultOps(self._projector_xform, [x, y, z], quat, [1, 1, 1])

    def randomizeCamera(self):
        """
        Randomizes the placement of the Camera."""

        attributes = self.mixer_camera.executeGraph(1)
        position = attributes["xformOp:translation"]
        orientation = attributes["xformOp:orientation"]
        setDefaultOps(UsdGeom.Xformable(self._camera_prim), position[0], orientation[0], (1, 1, 1))

    def switchTerrain(self, flag: int) -> None:
        super().switch_terrain(flag)
        self.createCameraSampler()
        self.randomizeCamera()

    def randomize(self) -> None:
        if self.counter % 100 == 0:
            self.randomize_rocks()
        if self.counter % 1000 == 0:
            self.switchTerrain(-1)
        self.randomizeProjector()
        self.randomizeCamera()
        self.counter += 1
