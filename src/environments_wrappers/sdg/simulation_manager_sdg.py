__author__ = "Antoine Richard"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"


import omni

from src.environments_wrappers.sdg.lunalab_sdg import SDG_Lunalab
from src.environments_wrappers.sdg.lunaryard_sdg import SDG_Lunaryard
from src.labeling.auto_label import AutonomousLabeling
from src.physics.simulation import Simulation


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
        self.sim = Simulation(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
        self.sim.physics_scene.set_solver_type("PGS")
        # Lab manager thread
        self.sim.reset()
        self.LC = SDG_LMF(cfg)
        self.LC.load()
        print("After lab is loaded")
        for i in range(100):
            self.sim.step()
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
            self.sim.step()
            if self.sim.is_playing():
                try:
                    self.AL.record()
                    self.count += 1
                except Exception:
                    pass
                self.LC.randomize()
        self.timeline.stop()
