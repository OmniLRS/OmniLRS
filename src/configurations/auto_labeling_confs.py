__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

import dataclasses


@dataclasses.dataclass
class AutoLabelingConf:
    """
    Args:
        num_images (int): The number of images to generate.
        prim_path (str): Path to the prim file.
        camera_names (list): Name of the cameras.
        camera_resolutions (list): Resolutions of the camera.
        data_dir (str): Path to the directory where the synthetic data will be saved.
        annotators_list (list): List of annotators. We currently support: "rgb", "instance", "semantic", "depth", "ir".
        image_format (str): Format of the images. For instance: "png", or "jpg".
        annot_format (str): Format of the annotations. For instance: "json".
        element_per_folder (int): Number of elements that will be saved per folder.
                                  This is done to avoid having too many files in a single folder.
        save_intrisics (bool): Whether to save the intrinsics of the camera.
    """

    num_images: int = dataclasses.field(default=int)
    prim_path: str = dataclasses.field(default_factory=str)
    camera_names: list = dataclasses.field(default_factory=list)
    camera_resolutions: list = dataclasses.field(default_factory=list)
    data_dir: str = dataclasses.field(default_factory=str)
    annotators_list: list = dataclasses.field(default_factory=list)
    image_formats: list = dataclasses.field(default_factory=list)
    annot_formats: list = dataclasses.field(default_factory=list)
    element_per_folder: int = dataclasses.field(default_factory=int)
    save_intrinsics: bool = dataclasses.field(default_factory=bool)

    def __post_init__(self):
        assert len(self.camera_names) > 0, "camera_names must have at least one element"
        assert len(self.camera_names) == len(
            self.camera_resolutions
        ), "camera_names and camera_resolutions must have the same length"
        assert len(self.camera_names) == len(
            self.annotators_list
        ), "camera_names and annotators_list must have the same length"
        assert len(self.camera_names) == len(
            self.image_formats
        ), "camera_names and image_format must have the same length"
        assert len(self.camera_names) == len(
            self.annot_formats
        ), "camera_names and annot_format must have the same length"

        for camera_name in self.camera_names:
            assert type(camera_name) is str, "camera_name must be a string"
        for camera_resolution in self.camera_resolutions:
            assert len(camera_resolution) == 2, "camera_resolution must be a tuple of length 2" 
        for annotator in self.annotators_list:
            assert type(annotator) is list, "annotator_list must be a list"
        for image_format in self.image_formats:
            assert type(image_format) is str, "image_format must be a string"
            assert image_format in [
                "png",
                "jpeg",
                "jpg",
                "tiff",
                "tif",
            ], "image_format must be png, jpg, jpeg, tiff or tif"
        for annot_format in self.annot_formats:
            assert type(annot_format) is str, "annot_format must be a string"
            assert annot_format in [
                "json",
                "csv",
                "yaml",
            ], "annot_format must be json, csv, or yaml"

        assert type(self.num_images) is int, "num images must be an integer"
        assert type(self.prim_path) is str, "prim_path must be a string"
        assert type(self.data_dir) is str, "data_dir must be a string"
        assert type(self.element_per_folder) is int, "element_per_folder must be an integer"

        assert self.num_images > 0, "num_images must be larger than 0"
        assert self.element_per_folder > 0, "element_per_folder must be greater than 0"
        assert self.data_dir != "", "data_dir cannot be empty"


@dataclasses.dataclass
class CameraConf:
    camera_path: str = "Camera/camera_annotations"
    focal_length: float = 1.93
    horizontal_aperture: float = 2.4
    vertical_aperture: float = 1.8
    fstop: float = 0.00
    focus_distance: float = 10.0
    clipping_range: tuple = (0.01, 1000000.0)

    def __post_init__(self):
        assert type(self.camera_path) is str, "camera_path must be a string"
        assert type(self.focal_length) is float, "focal_length must be a float"
        assert type(self.horizontal_aperture) is float, "horizontal_aperture must be a float"
        assert type(self.vertical_aperture) is float, "vertical_aperture must be a float"
        assert type(self.fstop) is float, "fstop must be a float"
        assert type(self.focus_distance) is float, "focus_distance must be a float"
        assert type(self.clipping_range) is tuple, "clipping_range must be a tuple"

        assert self.camera_path != "", "camera_path must not be empty"
        assert len(self.clipping_range) == 2, "clipping_range must be a tuple of length 2"
        assert self.focal_length > 0, "focal_length must be greater than 0"
        assert self.focus_distance > 0, "focusDistance must be greater than 0"
        assert self.horizontal_aperture > 0, "horizontal_aperture must be greater than 0"
        assert self.vertical_aperture > 0, "vertical_aperture must be greater than 0"
        assert self.fstop >= 0, "fstop must be greater or equal to 0"
        assert (
            self.clipping_range[1] > self.clipping_range[0]
        ), "clipping_range[1] must be greater than clipping_range[0]"
