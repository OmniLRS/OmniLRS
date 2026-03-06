__author__ = "Antoine Richard, Junnosuke Kahamora"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Any, Union, Dict
import pandas as pd
import numpy as np
import json
import cv2
import os


class BaseWriter:
    """
    Base class for writers.
    """

    def __init__(
        self, root_path: str = None, name: str = None, element_per_folder: int = 1000, prefix: str = "", **kwargs
    ) -> None:
        """
        Initialize the BaseWriter class.

        Args:
            root_path (str): The root path of the data.
            name (str): The name of the data.
            element_per_folder (int, optional): The number of elements per folder. Defaults to 1000.
            prefix (str, optional): The preffix of the data. Defaults to "".
        """

        self.root_path = root_path
        self.data_path = os.path.join(root_path, prefix + name)
        self.element_per_folder = element_per_folder
        self.counter = 0
        self.folder_counter = 0
        self.image_fix_name_size = len(str(element_per_folder))

    def makeFolder(self) -> None:
        """
        Make a folder to store the data.
        """

        if self.counter % self.element_per_folder == 0:
            self.current_folder = os.path.join(self.data_path, str(self.folder_counter))
            os.makedirs(self.current_folder, exist_ok=True)
            self.counter = 0
            self.folder_counter += 1

    def write(self, data: np.ndarray, **kwargs) -> None:
        """
        Write the data to a file.

        Args:
            data (np.ndarray): The data to write.
            **kwargs: Additional arguments.
        """

        raise NotImplementedError


class WritePoseData(BaseWriter):
    """
    Write pose data to a file.
    """

    def __init__(
        self,
        root_path: str,
        name: str = "pose",
        format: str = "csv",
        prefix: str = "",
        **kwargs,
    ) -> None:
        """
        Initialize the WritePoseData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "pose".
            format (str, optional): The format of the data. Defaults to "csv".
            prefix (str, optional): The prefix of the data. Defaults to "".
            **kwargs: Additional arguments.
        """

        super().__init__(root_path, name=name, prefix=prefix)
        self.format = format
        self.path_to_data = os.path.join(self.data_path, name + "." + format)

        os.makedirs(self.data_path, exist_ok=True)

    def write(self, data: Dict[str, Any], **kwargs) -> None:
        """
        Write pose data to a file.

        Args:
            data (Dict): The pose data.
            **kwargs: Additional arguments.
        """

        if os.path.exists(self.path_to_data):
            if self.format == "csv":
                old_data = pd.read_csv(self.path_to_data, index_col=0)
            else:
                raise NotImplementedError(f"Format {self.format} not implemented")

        if self.format == "csv":
            new_data = pd.DataFrame(data)
            new_data.index = [self.counter]
            if os.path.exists(self.path_to_data):
                new_data = pd.concat([old_data, new_data])
            new_data.to_csv(self.path_to_data)

        else:
            raise NotImplementedError(f"Format {self.format} not implemented")
        self.counter += 1


class WriteRGBData(BaseWriter):
    """
    Write RGB data to a file.
    """

    def __init__(
        self,
        root_path: str,
        name: str = "rgb",
        prefix: str = "",
        element_per_folder: int = 1000,
        image_format: str = "png",
        **kwargs,
    ) -> None:
        """
        Initialize the WriteRGBData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "rgb".
            prefix (str, optional): The prefix of the data. Defaults to "".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            image_format (str, optional): The image format. Defaults to "png".
        """

        super().__init__(root_path, name=name, prefix=prefix, element_per_folder=element_per_folder)
        self.image_format = image_format

    def write(self, data: np.ndarray, **kwargs) -> None:
        """
        Write RGB data to a file.

        Args:
            data (np.ndarray): The RGB data.
            **kwargs: Additional arguments.
        """

        rgb_image = np.frombuffer(data, dtype=np.uint8).reshape(*data.shape, -1)
        rgb_image = np.squeeze(rgb_image)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGBA2BGRA)
        self.makeFolder()
        cv2.imwrite(
            os.path.join(
                self.current_folder,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.image_format,
            ),
            rgb_image,
        )
        self.counter += 1


class WriteIRData(BaseWriter):
    """
    Write RGB data to a file.
    """

    def __init__(
        self,
        root_path: str,
        name: str = "ir",
        prefix: str = "",
        element_per_folder: int = 1000,
        image_format: str = "png",
        **kwargs,
    ) -> None:
        """
        Initialize the WriteRGBData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "rgb".
            prefix (str, optional): The prefix of the data. Defaults to "".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            image_format (str, optional): The image format. Defaults to "png".
            seed (int, optional): The seed used to generate the random numbers. Defaults to 42.
        """

        super().__init__(root_path, name=name, prefix=prefix, element_per_folder=element_per_folder)
        self.image_format = image_format

    def write(self, data: np.ndarray, **kwargs) -> None:
        """
        Write RGB data to a file.

        Args:
            data (np.ndarray): The RGB data.
            **kwargs: Additional arguments.
        """

        rgb_image = np.frombuffer(data, dtype=np.uint8).reshape(*data.shape, -1)
        rgb_image = np.squeeze(rgb_image)
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
        self.makeFolder()
        cv2.imwrite(
            os.path.join(
                self.current_folder,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.image_format,
            ),
            rgb_image,
        )
        self.counter += 1


class WriteDepthData(BaseWriter):
    """
    Write Depth data to a file.
    """

    def __init__(
        self, root_path: str, name: str = "depth", prefix: str = "", element_per_folder: int = 1000, **kwargs
    ) -> None:
        """
        Initialize the WriteRGBData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "depth".
            prefix (str, optional): The prefix of the data. Defaults to "".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            image_format (str, optional): The image format. Defaults to "png".
        """

        image_format = "npz"
        super().__init__(root_path, name=name, prefix=prefix, element_per_folder=element_per_folder)
        self.image_format = image_format

    def write(self, data: np.ndarray, **kwargs) -> None:
        """
        Write Depth data to a file.

        Args:
            data (np.ndarray): The Depth data.
            **kwargs: Additional arguments.
        """

        depth_image = np.frombuffer(data, dtype=np.float32).reshape(*data.shape, -1)
        depth_image = np.squeeze(depth_image)
        self.makeFolder()
        np.savez_compressed(
            os.path.join(self.current_folder, f"{self.counter:0{self.image_fix_name_size}d}." + self.image_format),
            depth=depth_image,
        )
        self.counter += 1


class WriteSemanticData(BaseWriter):
    """
    Write semantic segmentation data to a file.
    """

    def __init__(
        self,
        root_path: str,
        name: str = "semantic_segmentation",
        prefix: str = "",
        element_per_folder: int = 1000,
        image_format: str = "png",
        annot_format: str = "json",
        **kwargs,
    ) -> None:
        """
        Initialize the WriteSemanticData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "semantic_segmentation".
            prefix (str, optional): The prefix of the data. Defaults to "".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            image_format (str, optional): The image format. Defaults to "png".
            annot_format (str, optional): The annotation format. Defaults to "json".
            **kwargs: Additional arguments.
        """

        super().__init__(root_path, name=name, prefix=prefix, element_per_folder=element_per_folder)
        self.image_format = image_format
        self.annot_format = annot_format

    def makeFolder(self) -> None:
        """
        Make a folder to store the data.
        """

        if self.counter % self.element_per_folder == 0:
            self.current_folder_image = os.path.join(self.data_path, str(self.folder_counter))
            self.current_folder_labels = os.path.join(self.data_path + "_id_label", str(self.folder_counter))
            os.makedirs(self.current_folder_image, exist_ok=True)
            os.makedirs(self.current_folder_labels, exist_ok=True)
            self.counter = 0
            self.folder_counter += 1

    def write(self, data: Dict[str, np.ndarray], **kwargs) -> None:
        """
        Write semantic segmentation data to a file.

        Args:
            data (Dict): The semantic segmentation data.
            **kwargs: Additional arguments.
        """

        id_to_labels = data["info"]["idToLabels"]
        sem_image_data = np.frombuffer(data["data"], dtype=np.uint8).reshape(*data["data"].shape, -1)
        sem_image_data = np.squeeze(sem_image_data)
        sem_image_data = cv2.cvtColor(sem_image_data, cv2.COLOR_RGBA2BGRA)
        self.makeFolder()
        with open(
            os.path.join(
                self.current_folder_labels,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.annot_format,
            ),
            "w",
        ) as f:
            json.dump(id_to_labels, f)
        cv2.imwrite(
            os.path.join(
                self.current_folder_image,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.image_format,
            ),
            sem_image_data,
        )
        self.counter += 1


class WriteInstanceData(BaseWriter):
    """
    Write instance semantic segmentation data to a file.
    """

    def __init__(
        self,
        root_path: str,
        name: str = "semantic_segmentation",
        prefix: str = "",
        element_per_folder: int = 10000,
        image_format: str = "png",
        annot_format: str = "json",
        **kwargs,
    ) -> None:
        """
        Initialize the WriteInstanceData class.

        Args:
            root_path (str): The root path of the data.
            name (str, optional): The name of the data. Defaults to "semantic_segmentation".
            prefix (str, optional): The prefix of the data. Defaults to "".
            element_per_folder (int, optional): The number of elements per folder. Defaults to 10000.
            image_format (str, optional): The image format. Defaults to "png".
            annot_format (str, optional): The annotation format. Defaults to "json".
            **kwargs: Additional arguments.
        """

        super().__init__(root_path, name=name, prefix=prefix, element_per_folder=element_per_folder)
        self.image_format = image_format
        self.annot_format = annot_format

    def makeFolder(self) -> None:
        """
        Make a folder to store the data.
        """

        if self.counter % self.element_per_folder == 0:
            self.current_folder_image = os.path.join(self.data_path, str(self.folder_counter))
            self.current_folder_labels = os.path.join(self.data_path + "_id_label", str(self.folder_counter))
            self.current_folder_semantics = os.path.join(self.data_path + "_id_semantic", str(self.folder_counter))
            os.makedirs(self.current_folder_image, exist_ok=True)
            os.makedirs(self.current_folder_labels, exist_ok=True)
            os.makedirs(self.current_folder_semantics, exist_ok=True)
            self.counter = 0
            self.folder_counter += 1

    def write(self, data: Dict[str, np.ndarray], **kwargs) -> None:
        """
        Write instance segmentation data to a file.

        Args:
            data (Dict): The instance segmentation data.
            **kwargs: Additional arguments.
        """

        id_to_labels = data["info"]["idToLabels"]
        id_to_semantic = data["info"]["idToSemantics"]
        instance_image_data = np.frombuffer(data["data"], dtype=np.uint8).reshape(*data["data"].shape, -1)
        instance_image_data = np.squeeze(instance_image_data)
        instance_image_data = cv2.cvtColor(instance_image_data, cv2.COLOR_RGBA2BGRA)
        self.makeFolder()
        with open(
            os.path.join(
                self.current_folder_labels,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.annot_format,
            ),
            "w",
        ) as f:
            json.dump(id_to_labels, f)
        with open(
            os.path.join(
                self.current_folder_semantics,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.annot_format,
            ),
            "w",
        ) as f:
            json.dump(id_to_semantic, f)
        cv2.imwrite(
            os.path.join(
                self.current_folder_image,
                f"{self.counter:0{self.image_fix_name_size}d}." + self.image_format,
            ),
            instance_image_data,
        )
        self.counter += 1


class WriterFactory:
    """
    A factory class to create writers.
    """

    def __init__(self):
        self.writers = {}

    def registerWriter(self, name: str, writer: BaseWriter, **kwargs):
        """
        Register a writer to the factory.

        Args:
            name (str): The name of the writer.
            writer (BaseWriter): The writer to register.
            **kwargs: Additional arguments.
        """

        self.writers[name] = writer

    def __call__(self, name: str = None, **kwargs) -> BaseWriter:
        """
        Create a writer.

        Args:
            name (str): The name of the writer.
            **kwargs: Additional arguments.

        Returns:
            BaseWriter: The writer.

        Raises:
            AssertionError: If the writer is not registered.
        """
        assert name in self.writers, f"Writer {name} not registered"
        return self.writers[name](**kwargs)


writerFactory = WriterFactory()
writerFactory.registerWriter("pose", WritePoseData)
writerFactory.registerWriter("rgb", WriteRGBData)
writerFactory.registerWriter("ir", WriteIRData)
writerFactory.registerWriter("depth", WriteDepthData)
writerFactory.registerWriter("semantic_segmentation", WriteSemanticData)
writerFactory.registerWriter("instance_segmentation", WriteInstanceData)
