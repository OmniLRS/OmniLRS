__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.environments_wrappers.sdg.lunaryard_sdg import SDG_Lunaryard
from src.environments_wrappers.sdg.lunalab_sdg import SDG_Lunalab
from src.configurations.auto_labeling_confs import AutoLabelingConf, CameraConf
from src.labeling.auto_label import AutonomousLabeling

from omni.isaac.core import World
from typing import Union
import omni


class SyntheticDataGeneration_LabManagerFactory:
    def __init__(self):
        self._lab_managers = {}

    def register(self, name, lab_manager):
        self._lab_managers[name] = lab_manager

    def __call__(self, cfg):
        return self._lab_managers[cfg["environment"]["name"]](
            **cfg["environment"],
            flares_settings=cfg["rendering"]["lens_flares"],
            camera_settings=cfg["mode"]["camera_settings"],
        )


SDG_LMF = SyntheticDataGeneration_LabManagerFactory()
SDG_LMF.register("Lunalab", SDG_Lunalab)
SDG_LMF.register("Lunaryard", SDG_Lunaryard)


class SDG_SimulationManager:
    """ "
    Manages the simulation. This class is responsible for:
    - Initializing the simulation
    - Running the simulation
    - Cleaning the simulation"""

    def __init__(self, cfg, simulation_app) -> None:
        self.simulation_app = simulation_app
        self.generation_settings = cfg["mode"]["generation_settings"]
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.physics_ctx = self.world.get_physics_context()
        self.physics_ctx.set_solver_type("PGS")
        # Lab manager thread
        self.world.reset()
        self.LC = SDG_LMF(cfg)
        self.LC.load()
        print("After lab is loaded")
        for i in range(100):
            self.world.step(render=True)
        print("After world reset")
        self.generation_settings.prim_path = self.LC.scene_name + "/" + self.generation_settings.prim_path
        self.AL = AutonomousLabeling(self.generation_settings)
        self.AL.load()
        self.count = 0
        # Randomize once to setup the camera mixer
        self.LC.randomize()


    def run_simulation(self) -> None:
        """
        Runs the simulation.
        """

        self.timeline.play()
        while self.simulation_app.is_running() and (self.count < self.generation_settings.num_images):
            self.world.step(render=True)
            if self.world.is_playing():
                try:
                    self.AL.record()
                    self.count += 1
                except:
                    pass
                self.LC.randomize()
        self.timeline.stop()
