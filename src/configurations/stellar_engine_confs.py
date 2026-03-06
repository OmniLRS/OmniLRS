__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

import dataclasses
import datetime
import os


@dataclasses.dataclass
class Date:
    year: int = 2024
    month: int = 5
    day: int = 11
    hour: int = 16
    minute: int = 30

    def __post_init__(self):
        self.year = int(self.year)
        self.month = int(self.month)
        self.day = int(self.day)
        self.hour = int(self.hour)
        self.minute = int(self.minute)

        assert 1 <= self.month <= 12, "Month must be between 1 and 12."
        assert 1 <= self.day <= 31, "Day must be between 1 and 31."
        assert 0 <= self.hour <= 23, "Hour must be between 0 and 23."
        assert 0 <= self.minute <= 59, "Minute must be between 0 and 59."
        assert 1970 <= self.year <= 2050, "Year must be between 1970 and 2050."


@dataclasses.dataclass
class StellarEngineConf:
    start_date: datetime.datetime = dataclasses.field(default_factory=dict)
    time_scale: float = 36000.0
    update_interval: float = 600.0
    ephemeris_path: str = "assets/Ephemeris"
    ephemeris: str = "de421.bsp"
    moon_pa: str = "moon_pa_de421_1900-2050.bpc"
    moon_tf: str = "moon_080317.tf"
    pck: str = "pck00008.tpc"
    frame: str = "MOON_ME_DE421"

    def __post_init__(self):
        if isinstance(self.start_date, dict):
            d = Date(**self.start_date)
        else:
            d = self.start_date
        self.start_date = datetime.datetime(d.year, d.month, d.day, d.hour, d.minute, tzinfo=datetime.timezone.utc)

        assert 0 < self.time_scale, "Time scale must be greater than 0."
        assert 0 < self.update_interval, "Update interval must be greater than 0."

        assert self.ephemeris in ["de421.bsp"], "Ephemeris must be one of 'de421.bsp'."
        assert self.moon_pa in ["moon_pa_de421_1900-2050.bpc"], "Moon PA must be one of 'moon_pa_de421_1900-2050.bpc'."
        assert self.moon_tf in ["moon_080317.tf"], "Moon TF must be one of 'moon_080317.tf'."
        assert self.pck in ["pck00008.tpc", "pck00011.tpc"], "PCK must be one of 'pck00008.tpc', 'pck00011.tpc'."
        assert self.frame in ["MOON_ME_DE421"], "Frame must be one of 'MOON_ME_DE421'."

        self.ephemeris = os.path.join(self.ephemeris_path, self.ephemeris)
        self.moon_pa = os.path.join(self.ephemeris_path, self.moon_pa)
        self.moon_tf = os.path.join(self.ephemeris_path, self.moon_tf)
        self.pck = os.path.join(self.ephemeris_path, self.pck)

        assert os.path.exists(self.ephemeris), f"Ephemeris file {self.ephemeris} does not exist."
        assert os.path.exists(self.moon_pa), f"Moon PA file {self.moon_pa} does not exist."
        assert os.path.exists(self.moon_tf), f"Moon TF file {self.moon_tf} does not exist."
        assert os.path.exists(self.pck), f"PCK file {self.pck} does not exist."


@dataclasses.dataclass
class SunConf:
    """
    Configuration for the sun light.

    Attributes:
        intensity (float): The intensity of the sun light.
        angle (float): The angle of the sun light. (in degrees)
        diffuse_multiplier (float): The diffuse multiplier of the sun light.
        specular_multiplier (float): The specular multiplier of the sun light.
        color (tuple): The color of the sun light.
        azimuth (float): The azimuth of the sun light. (in degrees)
        elevation (float): The elevation of the sun light. (in degrees)
    """

    intensity: float = 1750.0
    angle: float = 0.53
    diffuse_multiplier: float = 1.0
    specular_multiplier: float = 1.0
    color: tuple = (1.0, 1.0, 1.0)
    temperature: float = 6500.0
    azimuth: float = 0.0
    elevation: float = 0.0

    def __post_init__(self):
        assert type(self.intensity) == float, "The intensity must be a float."
        assert type(self.angle) == float, "The angle must be a float."
        assert type(self.diffuse_multiplier) == float, "The diffuse multiplier must be a float."
        assert type(self.specular_multiplier) == float, "The specular multiplier must be a float."
        assert type(self.temperature) == float, "The temperature must be a float."
        assert all([type(i) == float for i in self.color]), "The color must be a tuple of 3 floats."
        assert type(self.azimuth) == float, "The azimuth must be a float."
        assert type(self.elevation) == float, "The elevation must be a float."

        assert self.intensity > 0.0, "The intensity must be greater than 0."
        assert self.angle > 0.0, "The angle must be greater than 0."
        assert self.diffuse_multiplier > 0.0, "The diffuse multiplier must be greater than 0."
        assert self.specular_multiplier > 0.0, "The specular multiplier must be greater than 0."
        assert len(self.color) == 3, "The color must be a tuple of 3 floats."
        assert self.temperature > 0.0, "The temperature must be greater than 0."
        assert all([i >= 0.0 and i <= 1.0 for i in self.color]), "The color must be between 0 and 1."
        assert self.azimuth >= 0.0 and self.azimuth <= 360.0, "The azimuth must be between 0 and 360 degrees."
        assert self.elevation >= 0.0 and self.elevation <= 360.0, "The elevation must be between 0 and 360 degrees."
