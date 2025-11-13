__author__ = "Antoine Richard, Junnosuke Kahamora"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from typing import List, Tuple, Dict, Union
import random, string
import numpy as np
import omni
import os

import omni.replicator.core as rep
import omni.kit.actions.core
from pxr import Usd, UsdGeom

from src.configurations.auto_labeling_confs import AutoLabelingConf
from src.labeling.rep_utils import writerFactory


class PoseAnnotator:
    def __init__(self, prim):
        self.prim = prim
        self.xform = UsdGeom.Xformable(prim)

    def get_data(self):
        time = Usd.TimeCode.Default()
        world_transform = self.xform.ComputeLocalToWorldTransform(time)
        translation = world_transform.ExtractTranslation()
        rotation = world_transform.ExtractRotationQuat()
        return {
            "position_x": [translation[0]],
            "position_y": [translation[1]],
            "position_z": [translation[2]],
            "quaternion_x": [rotation.GetImaginary()[0]],
            "quaternion_y": [rotation.GetImaginary()[1]],
            "quaternion_z": [rotation.GetImaginary()[2]],
            "quaternion_w": [rotation.GetReal()],
        }


class AutonomousLabeling:
    """
    AutonomousLabeling class. It is used to generate synthetic data from a USD scene.
    """

    def __init__(
        self,
        cfg: AutoLabelingConf,
        **kwargs,
    ) -> None:
        """
        Initialize the AutonomousLabeling class.

        Args:
            cfg: Configuration for the autonomous labeling.
        """

        # Camera parameters
        self.camera_names = cfg.camera_names
        self.camera_resolutions = cfg.camera_resolutions
        self.save_camera_intrinsics = cfg.save_intrinsics

        # Data storage parameters
        self.data_hash = "".join(random.sample(string.ascii_letters + string.digits, 16))
        self.data_dir = os.path.join(cfg.data_dir, self.data_hash)
        self.camera_prims = {}
        self.camera_paths = {}
        self.render_products = {}
        self.annotators = {}
        self.writers_cfgs = {}
        self.synthetic_writers = {}

        # Synthetic data parameters
        self.annotators_list = cfg.annotators_list
        self.image_formats = cfg.image_formats
        self.annot_formats = cfg.annot_formats
        self.element_per_folder = cfg.element_per_folder
        self.formatWriterConfig()
        self.loggers = {
            "pose": self.enablePose,
            "rgb": self.enableRGBData,
            "instance_segmentation": self.enableInstanceData,
            "semantic_segmentation": self.enableSemanticData,
            "depth": self.enableDepthData,
            "ir": self.enableIRData,
        }

        self.stage = omni.usd.get_context().get_stage()
        self.meta_prim = self.stage.GetPrimAtPath(cfg.prim_path)
        self.synth_counter = 0

        ## Verify the directory exists
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

    def formatWriterConfig(self) -> Dict[str, Union[float, bool, int, str]]:
        for i, camera_name in enumerate(self.camera_names):
            self.writers_cfgs[camera_name] = {}
            self.synthetic_writers[camera_name] = {}
            for name in self.annotators_list[i]:
                self.writers_cfgs[camera_name][name] = {}
                self.writers_cfgs[camera_name][name]["root_path"] = self.data_dir
                self.writers_cfgs[camera_name][name]["prefix"] = camera_name + "_"
                self.writers_cfgs[camera_name][name]["element_per_folder"] = self.element_per_folder
                self.writers_cfgs[camera_name][name]["image_format"] = self.image_formats[i]
                self.writers_cfgs[camera_name][name]["annot_format"] = self.annot_formats[i]
                self.synthetic_writers[camera_name][name] = writerFactory(name, **self.writers_cfgs[camera_name][name])

    def get_intrinsics_matrix(self, camera_prim) -> np.ndarray:
        """
        Returns:
            np.ndarray: the intrinsics of the camera (used for calibration)
        """
        focal_length = camera_prim.GetAttribute("focalLength").Get() / 10.0
        (width, height) = (1280, 720)
        horizontal_aperture = camera_prim.GetAttribute("horizontalAperture").Get() / 10.0
        vertical_aperture = (camera_prim.GetAttribute("horizontalAperture").Get() / 10.0) * (float(height) / width)
        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture
        cx = width * 0.5
        cy = height * 0.5
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype="float32")

    def save_intrinsics(self, camera_name: str) -> None:
        intrinsics = self.get_intrinsics_matrix(self.camera_prims[camera_name])
        np.savetxt(os.path.join(self.data_dir, camera_name + "_intrisics.csv"), intrinsics, delimiter=",")
        np.save(os.path.join(self.data_dir, camera_name + "_intrisics.npy"), intrinsics)

    def findCameraForAnnotation(self) -> None:
        """
        Find the prim and path of the cameras that will be used to generate the synthetic data.
        """
        for prim in Usd.PrimRange(self.meta_prim):
            if prim.GetName() in self.camera_names:
                self.camera_prims[prim.GetName()] = prim
                self.camera_paths[prim.GetName()] = str(prim.GetPath())

        if len(self.camera_prims) != len(self.camera_names):
            raise ValueError("Some cameras were not found in the scene")

    def get_camera_idx(self, camera_name: str) -> int:
        idx = -1
        for i, name in enumerate(self.camera_names):
            if name == camera_name:
                idx = i
        if idx == -1:
            raise ValueError("Camera not found")
        return idx

    def load(self) -> None:
        """
        Finds the cameras and enables the collection of data from them.
        """

        self.findCameraForAnnotation()
        for camera_name in self.camera_names:
            if self.save_camera_intrinsics:
                self.save_intrinsics(camera_name)
            idx = self.get_camera_idx(camera_name)
            self.setCamera(self.camera_paths[camera_name], camera_name, self.camera_resolutions[idx])
            for annotator in self.annotators_list[idx]:
                self.loggers[annotator](camera_name)

    def setCamera(self, camera_path: str, camera_name: str, res: Tuple[float, float] = (640, 480)) -> None:
        """
        Set the camera resolution that will be used to generate the synthetic data.

        Args:
            camera_path (str): The path to the camera.
            res (tuple, optional): The resolution of the camera. Defaults to (640, 480).
        """

        self.render_products[camera_name] = rep.create.render_product(camera_path, res)

    def enablePose(self, camera_name: str) -> None:
        """
        Enable the collection of pose data.
        """
        pose_annot = PoseAnnotator(self.camera_prims[camera_name])
        self.annotators[camera_name + "_pose"] = (camera_name, "pose", pose_annot)

    def enableRGBData(self, camera_name: str) -> None:
        """
        Enable the collection of RGB data.
        """

        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotators[camera_name + "_rgb"] = (camera_name, "rgb", rgb_annot)
        rgb_annot.attach([self.render_products[camera_name]])

    def enableIRData(self, camera_name: str) -> None:
        """
        Enable the collection of RGB data.
        """

        ir_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotators[camera_name + "_ir"] = (camera_name, "ir", ir_annot)
        ir_annot.attach([self.render_products[camera_name]])

    def enableDepthData(self, camera_name: str) -> None:
        """
        Enable the collection of RGB data.
        """

        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self.annotators[camera_name + "_depth"] = (camera_name, "depth", depth_annot)
        depth_annot.attach([self.render_products[camera_name]])

    def enableSemanticData(self, camera_name: str) -> None:
        """
        Enable the collection of semantic segmentation data.
        """

        semantic_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation", init_params={"colorize": True})
        self.annotators[camera_name + "_semantic_segmentation"] = (camera_name, "semantic_segmentation", semantic_annot)
        semantic_annot.attach([self.render_products[camera_name]])

    def enableInstanceData(self, camera_name: str) -> None:
        """
        Enable the collection of instance segmentation data.
        """

        instance_annotator = rep.AnnotatorRegistry.get_annotator(
            "instance_segmentation", init_params={"colorize": True}
        )
        self.annotators[camera_name + "_instance_segmentation"] = (
            camera_name,
            "instance_segmentation",
            instance_annotator,
        )
        instance_annotator.attach([self.render_products[camera_name]])

    def record(self) -> None:
        """
        Record a frame of synthetic data.
        """

        for camera_name, name, annotator in self.annotators.values():
            self.synthetic_writers[camera_name][name].write(annotator.get_data())
        self.synth_counter += 1
