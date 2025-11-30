__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from WorldBuilders.Types import *
from WorldBuilders.Mixer import RequestMixer
from typing import Any, Union, List, Dict
from WorldBuilders.pxr_utils import createInstancerAndCache, setInstancerParameters
from src.labeling.instancer import CustomInstancer
from assets import get_assets_path
import omni
import os


class TypeFactory:
    """
    TypeFactory class. It allows to register types and create objects of the registered types.
    """

    def __init__(self):
        self.types = {}

    def register_type(
        self,
        type_name: str,
        worldbuilder_type,
    ) -> None:
        """
        Register a type.

        Args:
            type_name (str): The name of the type.
            worldbuilder_type (WorldBuilder): The type of the worldbuilder.
        """

        self.types[type_name] = worldbuilder_type

    def __call__(
        self,
        name: str = None,
        **kwargs,
    ) -> Any:
        """
        Create an object of a specific type.

        Args:
            name (str, optional): The name of the type. Defaults to None.
            **kwargs: The arguments of the type.
        """

        if name is None:
            raise ValueError("Incorrect type name passed. Available types are: {}".format(self.types.keys()))
        else:
            return self.types[name](**kwargs)


samplerTypeFactory = TypeFactory()
samplerTypeFactory.register_type("Uniform", UniformSampler_T)
samplerTypeFactory.register_type("HardCoreUniform", HardCoreUniformSampler_T)
samplerTypeFactory.register_type("Normal", NormalSampler_T)
samplerTypeFactory.register_type("MaternCluster", MaternClusterPointSampler_T)
samplerTypeFactory.register_type("HardCoreMaternCluster", HardCoreMaternClusterPointSampler_T)
samplerTypeFactory.register_type("Poisson", PoissonPointSampler_T)
samplerTypeFactory.register_type("LinearInterpolation", LinearInterpolationSampler_T)
samplerTypeFactory.register_type("ThomasCluster", ThomasClusterSampler_T)
samplerTypeFactory.register_type("HardCoreThomasCluster", HardCoreThomasClusterSampler_T)
samplerTypeFactory.register_type("DeterministicSampler", DeterministicSampler_T)
samplerTypeFactory.register_type("Image", ImageClipper_T)
samplerTypeFactory.register_type("NormalMap", NormalMapClipper_T)

attributeFactory = TypeFactory()
attributeFactory.register_type("Position", Position_T)
attributeFactory.register_type("Orientation", Orientation_T)
attributeFactory.register_type("Scale", Scale_T)

layerFactory = TypeFactory()
layerFactory.register_type("Line", Line_T)
layerFactory.register_type("Circle", Circle_T)
layerFactory.register_type("plane", Plane_T)
layerFactory.register_type("Disk", Disk_T)
layerFactory.register_type("Cube", Cube_T)
layerFactory.register_type("Cylinder", Cylinder_T)
layerFactory.register_type("Sphere", Sphere_T)
layerFactory.register_type("Cone", Cone_T)
layerFactory.register_type("Torus", Torus_T)
layerFactory.register_type("Image", Image_T)
layerFactory.register_type("Ejecta", Image_T)
layerFactory.register_type("NormalMap", NormalMap_T)
layerFactory.register_type("RollPitchYaw", RollPitchYaw_T)


def requestGenerator(request: dict) -> Any:
    """
    Generates a request.

    Args:
        request (dict): The request.

    Returns:
        Any: The request.
    """

    attr = attributeFactory(request["attribute"])
    layer = layerFactory(**request["layer"])
    sampler = samplerTypeFactory(**request["sampler"])
    return UserRequest_T(p_type=attr, layer=layer, sampler=sampler, axes=request["axes"])


def getMixer(requests: dict) -> RequestMixer:
    """
    Get a mixer.

    Args:
        requests (dict): The requests.

    Returns:
        RequestMixer: The mixer.
    """

    return RequestMixer([requestGenerator(request) for request in requests.values()])


def addImageData(requests: dict, image: np.ndarray, mask: np.ndarray = None, ejecta_mask: np.ndarray = None) -> dict:
    """
    Adds image data to the requests.

    Args:
        requests (dict): The requests.
        image (np.ndarray): The image.
        mask (np.ndarray): The mask.

    Returns:
        dict: The requests.
    """

    for name in requests.keys():
        if requests[name]["layer"]["name"] == "Image":
            requests[name]["layer"]["data"] = mask
        elif requests[name]["layer"]["name"] == "Ejecta":
            requests[name]["layer"]["data"] = ejecta_mask
        if requests[name]["sampler"]["name"] in ["Image", "ClipMap", "Geoclipmap"]:
            requests[name]["sampler"]["data"] = image
            requests[name]["sampler"]["resolution"] = image.shape[:2]
    return requests


def generateMixer(requests: dict, image: np.ndarray, mask: np.ndarray, ejecta_mask: np.ndarray) -> RequestMixer:
    """
    Generates a mixer.

    Args:
        requests (dict): The requests.
        image (np.ndarray): The image.
        mask (np.ndarray): The mask.

    Returns:
        RequestMixer: The mixer.
    """
    requests = addImageData(requests, image, mask, ejecta_mask)
    return getMixer(requests)


class OGInstancer:
    """
    The Original Gangster: the point instancer.
    """

    def __init__(self, instancer_path, asset_list, seed):
        self.instancer_path = instancer_path
        self.stage = omni.usd.get_context().get_stage()
        self.prototypes = asset_list
        createInstancerAndCache(self.stage, self.instancer_path, self.prototypes)
        self.rng = np.random.default_rng(seed=seed)

    def setInstanceParameter(self, position: np.ndarray, orientation: np.ndarray, scale: np.ndarray, **kwargs) -> None:
        """
        Set the instancer's parameters. It sets the position, orientation, and scale of the instances.

        Args:
            position (np.ndarray): The position of the instances.
            orientation (np.ndarray): The orientation of the instances.
            scale (np.ndarray): The scale of the instances.
            **kwargs: Extra arguments.
        """

        ids = self.rng.integers(0, len(self.prototypes), position.shape[0])
        setInstancerParameters(
            self.stage,
            self.instancer_path,
            position,
            quat=orientation,
            scale=scale,
            ids=ids,
            **kwargs,
        )


class RockManager:
    """
    The RockManager class. It manages the rocks in the environment.
    """

    def __init__(self, rocks_settings: dict = None, instancers_path: str = None, enable: bool = True, **kwargs) -> None:
        """
        Args:
            rocks_settings (dict, optional): The settings of the rocks. Defaults to None.
            instancers_path (str, optional): The instancers path. Defaults to None.
            enable (bool, optional): The enable flag. Defaults to True.
        """
        self.stage = omni.usd.get_context().get_stage()

        self.instancers = {}
        self.settings = {}
        self.mixers = {}

        self.instancers_path = instancers_path
        self.mappings = {
            "xformOp:translation": "position",
            "xformOp:orientation": "orientation",
            "xformOp:scale": "scale",
        }

        for name, settings in rocks_settings.items():
            self.settings[name] = settings

        self.enable = enable
        if self.enable:
            # This is probalby over-engineered (like every good things in life)
            # Creates a dependency graph to know which node depends on which.
            # It also collects the root nodes, the ones that don't depend on any.
            # As well as all the nodes.
            self.dependency_graph = {}
            self.root_nodes = []
            self.nodes = []
            self.buildDependencyGraph()
            self.children_nodes = [i for l in self.dependency_graph.values() for i in l]
            # From there we compute the order in which the nodes need to be executed.
            self.execution_order = []
            self.buildExecutionOrder()

    def buildDependencyGraph(self) -> None:
        """
        Reads the configuration files and figures out which node (a node is a single bundle of requests) depends on which.
        It collects the root nodes, the ones that do not depend on any others.
        As well as builds a list of all the existing nodes.
        """

        for name, settings in self.settings.items():
            self.nodes.append(name)
            if "parent" in settings.keys():
                if not settings["parent"] is None:
                    assert (
                        settings["parent"] in self.settings.keys()
                    ), "the name of the parent must match the name of an existing process."
                    self.settings[settings["parent"]]["is_parent"] = True
                    if settings["parent"] in self.dependency_graph.keys():
                        self.dependency_graph[settings["parent"]].append(name)
                    else:
                        self.settings[settings["parent"]]["children"] = [name]
            else:
                self.root_nodes.append(name)
                if not name in self.dependency_graph.keys():
                    self.dependency_graph[name] = []

    def findPath(self, start: str, end: str, path: List[str] = []) -> Union[None, List[str]]:
        """
        Finds if the starting node, and ending nodes are connected.
        If so, returns the path between them, if not returns None.

        Args:
            start (str): The starting node.
            end (str): The ending node.
            path (List[str]): The list of nodes.

        Returns:
            Union[None, List[str]]
        """

        path = path + [start]
        if start == end:
            return path
        if not start in self.dependency_graph.keys():
            return None
        for node in self.dependency_graph[start]:
            if node not in path:
                newpath = self.findPath(node, end, path)
                if newpath:
                    return newpath
        return None

    def buildExecutionOrder(self) -> None:
        """
        Figures out in which order the nodes should be ran such that the depencies are satisfied.
        """

        paths = []
        # Makes a list of all the paths between the root nodes and all the other nodes.
        # It should be noted the cycles should not exist but we are not checking for them.
        # Also, a node should not have more than 1 parent.
        # It's up to the user to specify something correct.
        for root_node in self.root_nodes:
            for node in self.nodes:
                paths.append(self.findPath(root_node, node))
        # Removes the Nans when the paths are not possible
        paths = [path for path in paths if path]
        # Sorts the paths by length, and keeps the last element.
        self.execution_order = [i[-1] for i in sorted(paths, key=lambda i: len(i))]

    def build(self, image: np.ndarray, mask: np.ndarray, ejecta_mask: np.ndarray) -> None:
        """
        Builds the rock manager.
        It first creates the mixers, and then creates the instancers.
        Finally the mixers are exectuted, and the instancers are set to the randomized parameters.

        Args:
            image (np.ndarray): The image data.
            mask (np.ndarray): The mask data."""
        if self.enable:
            self.createInstancers()

    def updateImageData(self, image: np.ndarray, mask: np.ndarray, ejecta_mask: np.ndarray) -> None:
        """
        Creates the mixers with the image and mask data loaded in.
        The image and mask are used to place assets in the scene.

        Args:
            image (np.ndarray): The image data.
            mask (np.ndarray): The mask data."""

        if self.enable:
            self.image = image
            for name, settings in self.settings.items():
                self.mixers[name] = generateMixer(settings["requests"], image, mask, ejecta_mask)

    def createInstancers(self) -> None:
        """
        Creates as many instancers as there are types of rocks.
        There are two types of instancers, point instancers and custom instancers.
        The point instancers are the original instancers from USD.
        The custom instancers are instancers that enable the use of synthetic data generation.
        """

        for name, settings in self.settings.items():
            rock_assets = []
            for collection in settings["collections"]:
                root = get_assets_path() + "/USD_Assets/rocks/" + collection
                assets = os.listdir(root)
                assets = [
                    [
                        os.path.join(root, asset, i)
                        for i in os.listdir(os.path.join(root, asset))
                        if i.split(".")[-1] == "usd"
                    ]
                    for asset in assets
                ]
                assets = [item for sublist in assets for item in sublist]
                rock_assets += assets

            self.stage.DefinePrim(self.instancers_path, "Xform")

            self.stage.DefinePrim(os.path.join(self.instancers_path, name), "Xform")
            if settings["use_point_instancer"]:
                self.instancers[name] = OGInstancer(
                    os.path.join(self.instancers_path, name, "instancer"),
                    rock_assets,
                    seed=settings["seed"],
                )
            else:
                self.instancers[name] = CustomInstancer(
                    os.path.join(self.instancers_path, name, "instancer"),
                    rock_assets,
                    semantic_class=settings["class"],
                    seed=settings["seed"],
                )

    def randomizeInstancers(self, num: int) -> None:
        """
        Runs the mixers, collects the randomized parameters, and sets them to the instancers.

        Args:
            num (int): The number of instances to generate.
        """
        if self.enable:
            parents = {}
            for name in self.execution_order:
                if name in self.children_nodes:
                    output = self.mixers[name].executeGraph(parents=parents[self.settings[name]["parent"]])
                else:
                    output = self.mixers[name].executeGraph(num, use_mask_area=False)
                # Check if it the node is the parent of any other node.
                if (name in self.dependency_graph.keys()) and (len(self.dependency_graph[name]) > 0):
                    # If so collects parent data from the mixer.
                    parents[name] = self.mixers[name].getParents()
                # Updates the instancer.
                output = {self.mappings[key]: value for key, value in output.items()}
                self.instancers[name].setInstanceParameter(**output)

    def setVisible(self, flag: bool) -> None:
        """
        Set the visibility of the instancer.

        Args:
            flag (bool): The visibility flag."""

        if self.enable:
            instancers_prim = self.stage.GetPrimAtPath(self.instancers_path)
            if flag:
                instancers_prim.GetAttribute("visibility").Set("visible")
            else:
                instancers_prim.GetAttribute("visibility").Set("invisible")
