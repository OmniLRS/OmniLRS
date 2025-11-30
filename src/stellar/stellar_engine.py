__author__ = "Antoine Richard"
__copyright__ = "Copyright 2023-24, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD 3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Antoine Richard"
__email__ = "antoine.richard@uni.lu"
__status__ = "development"

from src.configurations.stellar_engine_confs import StellarEngineConf
from scipy.spatial.transform import Rotation as SSTR
from skyfield.api import PlanetaryConstants, load
from typing import Tuple
import datetime
import math
import logging


class StellarEngine:
    """
    StellarEngine class to compute the positions of celestial bodies.
    After initializing the StellarEngine, set the latitude and longitude of the observer using set_latlon method.
    Then, call the update method to update the current time.
    Finally, use the get_altaz, get_radec, get_position, or get_local_position methods to get the position of a given body.
    """

    def __init__(self, cfg: StellarEngineConf) -> None:
        """
        Initialize the StellarEngine.

        Args:
            cfg (StellarEngineConfig): the configuration for the StellarEngine.
        """

        self.cfg = cfg
        self.ts = load.timescale()
        self.current_time = cfg.start_date
        self.load_ephemeris()
        self.last_update = datetime.datetime.fromtimestamp(0, datetime.timezone.utc)
        self.t = self.ts.from_datetime(self.current_time)

    def load_ephemeris(self) -> None:
        """
        Load the ephemeris and planetary constants.
        """

        self.eph = load(self.cfg.ephemeris)
        self.earth, self.moon, self.sun, self.venus = (
            self.eph["earth"],
            self.eph["moon"],
            self.eph["sun"],
            self.eph["venus"],
        )
        self.bodies = {"earth": self.earth, "moon": self.moon, "sun": self.sun, "venus": self.venus}
        self.pc = PlanetaryConstants()
        self.pc.read_text(load(self.cfg.moon_tf))
        self.pc.read_text(load(self.cfg.pck))
        self.pc.read_binary(load(self.cfg.moon_pa))
        self.frame = self.pc.build_frame_named(self.cfg.frame)

    def set_lat_lon(self, lat: float, lon: float) -> None:
        """
        Set the latitude and longitude of the observer.

        Args:
            lat (float): the latitude of the observer.
            lon (float): the longitude of the observer.
        """

        self.observer = self.moon + self.pc.build_latlon_degrees(self.frame, lat, lon)

    def set_time(self, date: float) -> None:
        """
        Set the current time of the observer.

        Args:
            date (float): The current time of the observer. The given is given in seconds in the UTC time zone.
        """
        self.last_update = datetime.datetime.fromtimestamp(date, datetime.timezone.utc) #added fix
        self.current_time = datetime.datetime.fromtimestamp(date, datetime.timezone.utc)
        self.t = self.ts.from_datetime(self.current_time)
        logging.debug(f"StellarEngine: current time set to {self.current_time} and date set to {date}")

    def set_time_scale(self, time_scale: float) -> None:
        """
        Set the time scale of the observer.

        Args:
            time_scale (float): the time scale of the observer.
        """

        self.cfg.time_scale = time_scale

    def get_alt_az(self, body: str) -> Tuple[float, float, float]:
        """
        Get the altitude, azimuth, and distance of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the altitude, azimuth, and distance of.

        Returns:
            Tuple[float, float, float]: the altitude, azimuth, and distance of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(self.bodies[body]).apparent()
        alt, az, distance = apparent.altaz()
        return alt.degrees, az.degrees, distance.m

    def get_radec(self, body: str) -> Tuple[float, float, float]:
        """
        Get the ra, dec, and distance of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the ra, dec, and distance of.

        Returns:
            Tuple[float, float, float]: the ra, dec, and distance of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(self.bodies[body]).apparent()
        ra, dec, distance = apparent.radec(epoch="date")
        return ra, dec, distance.m

    def get_position(self, body: str) -> Tuple[float, float, float]:
        """
        Get the position of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the position of.

        Returns:
            Tuple[float, float, float]: the x, y, z position of the body in the observer's frame.
        """

        apparent = self.observer.at(self.t).observe(self.bodies[body]).apparent()
        return apparent.position.to("m").value

    def get_local_position(self, body: str) -> Tuple[float, float, float]:
        """
        Get the local position of the body in the observer's frame.

        Args:
            body (VectorSum): the body to get the position of.

        Returns:
            Tuple[float, float, float]: the x, y, z position of the body in the observer's frame.
        """

        alt, az, dist = self.get_alt_az(body)
        xyz = (
            dist * math.cos(math.radians(alt)) * math.cos(math.radians(az)),
            dist * math.cos(math.radians(alt)) * math.sin(math.radians(az)),
            dist * math.sin(math.radians(alt)),
        )
        return xyz

    def update(self, dt: float) -> bool:
        """
        Update the current time by dt seconds.
        If the time since the last update is greater than the update interval, update stellar time.
        Note that the delta of time is scaled by the time scale.

        Args:
            dt (float): time in seconds to update the current time.

        Returns:
            bool: True if the time was updated, False otherwise.
        """
        self.current_time += datetime.timedelta(seconds=dt * self.cfg.time_scale)
        # logging.debug(
        #     f"StellarEngine: current time is {self.current_time} (dt = {dt * self.cfg.time_scale}s)"
        # )
        time_delta = self.current_time - self.last_update
        update = False
        if time_delta.total_seconds() >= self.cfg.update_interval:
            self.last_update = self.current_time
            self.t = self.ts.from_datetime(self.current_time)
            update = True
            logging.debug(
                f"StellarEngine: updated time to {self.current_time} (dt = {time_delta.total_seconds()}s)"
            )

        return update

    def convert_alt_az_to_quat(self, alt: float, az: float) -> Tuple[float, float, float, float]:
        """
        Convert the altitude and azimuth to a quaternion.
        It is assumed that the default direction of the light is [0, 0, -1].
        The computed quaternion will rotate the light to the given altitude and azimuth.

        Args:
            alt (float): the altitude in degrees.
            az (float): the azimuth in degrees.

        Returns:
            Tuple[float, float, float, float]: the quaternion representing the altitude and azimuth. (qw, qx, qy, qz)
        """
        x, y, z, w = SSTR.from_euler("xyz", [0, alt, az - 90], degrees=True).as_quat()
        return (w, x, y, z)


if __name__ == "__main__":
    from src.configurations.stellar_engine_confs import Date
    from matplotlib import pyplot as plt
    import numpy as np

    date = Date(year=2024, month=5, day=1, hour=11, minute=50)
    SEC = StellarEngineConf(start_date=date, time_scale=1, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_lat_lon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing positions...")
    earth_pos = []
    sun_pos = []
    venus_pos = []
    for i in range(10000):
        SC.update(600)
        earth_pos.append(SC.get_position("earth"))
        sun_pos.append(SC.get_position("sun"))
        venus_pos.append(SC.get_position("venus"))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_pos = np.array(earth_pos)
    sun_pos = np.array(sun_pos)
    venus_pos = np.array(venus_pos)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(earth_pos[:, 0], earth_pos[:, 1], earth_pos[:, 2], label="Earth")
    ax.plot(sun_pos[:, 0], sun_pos[:, 1], sun_pos[:, 2], label="Sun")
    ax.plot(venus_pos[:, 0], venus_pos[:, 1], venus_pos[:, 2], label="Venus")
    plt.legend()

    date = Date(year=2019, month=12, day=20, hour=11, minute=5)
    SEC = StellarEngineConf(start_date=date, time_scale=1, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_lat_lon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing altaz...")
    earth_altaz = []
    sun_altaz = []
    venus_altaz = []
    for i in range(10000):
        SC.update(600)
        earth_altaz.append(SC.get_alt_az("earth"))
        sun_altaz.append(SC.get_alt_az("sun"))
        venus_altaz.append(SC.get_alt_az("venus"))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_altaz = np.array(earth_altaz)
    sun_altaz = np.array(sun_altaz)
    venus_altaz = np.array(venus_altaz)
    fig = plt.figure()
    plt.plot(earth_altaz[:, 0], label="Earth")
    plt.plot(sun_altaz[:, 0], label="Sun")
    plt.plot(venus_altaz[:, 0], label="Venus")
    plt.title("Altitude (degrees)")
    plt.legend()
    fig = plt.figure()
    plt.plot(earth_altaz[:, 1], label="Earth")
    plt.plot(sun_altaz[:, 1], label="Sun")
    plt.plot(venus_altaz[:, 1], label="Venus")
    plt.title("Azimuth (degrees)")
    plt.legend()

    date = Date(year=2019, month=12, day=20, hour=11, minute=5)
    SEC = StellarEngineConf(start_date=date, time_scale=1, update_interval=1)
    SC = StellarEngine(cfg=SEC)
    SC.set_lat_lon(-26.3, 46.8)

    start_time = datetime.datetime.now()
    print("Computing positions...")
    earth_pos = []
    sun_pos = []
    venus_pos = []
    for i in range(10000):
        SC.update(600)
        earth_pos.append(SC.get_local_position("earth"))
        sun_pos.append(SC.get_local_position("sun"))
        venus_pos.append(SC.get_local_position("venus"))
    end_time = datetime.datetime.now()

    print("Time taken:", (end_time - start_time).total_seconds(), "seconds")
    print("iterations per second:", 10000 / (end_time - start_time).total_seconds())

    # 3D plot of the positions
    earth_pos = np.array(earth_pos)
    sun_pos = np.array(sun_pos)
    venus_pos = np.array(venus_pos)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(earth_pos[:, 0], earth_pos[:, 1], earth_pos[:, 2], label="Earth")
    ax.plot(sun_pos[:, 0], sun_pos[:, 1], sun_pos[:, 2], label="Sun")
    ax.plot(venus_pos[:, 0], venus_pos[:, 1], venus_pos[:, 2], label="Venus")
    plt.legend()
    plt.show()
