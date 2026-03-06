__author__ = "Antoine Richard, Junnosuke Kamohara"
__copyright__ = "Copyright 2023-26, JAOPS, Space Robotics Lab, SnT, University of Luxembourg, SpaceR"
__license__ = "BSD-3-Clause"
__version__ = "2.0.0"
__maintainer__ = "Louis Burtz"
__email__ = "ljburtz@jaops.com"
__status__ = "development"

from typing import Tuple
import carb
import omni

from src.configurations.rendering_confs import FlaresConf, ChromaticAberrationsConf, MotionBlurConf


# ==============================================================================
# Renderer Control
# ==============================================================================


def enable_RTX_real_time(**kwargs) -> None:
    """
    Enables the RTX real time renderer. The ray-traced render.

    Args:
        data (int, optional): Not used. Defaults to 0.
    """

    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_realtime")
    action.execute()


def enable_RTX_interactive(*kwargs) -> None:
    """
    Enables the RTX interactive renderer. The path-traced render.

    Args:
        data (int, optional): Not used. Defaults to 0.
    """

    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.actions", "set_renderer_rtx_pathtracing")
    action.execute()


# ==============================================================================
# Lens Flares Controls
# ==============================================================================


def set_lens_flares(cfg) -> None:
    """
    Sets the lens flare settings.
    """

    if "lens_flares" in cfg["rendering"].keys():
        flare_cfg = cfg["rendering"]["lens_flares"]
        apply_lens_flare(flare_cfg)
    else:
        flare_cfg = None


def apply_lens_flare(settings: FlaresConf = None) -> None:
    """
    Enables the lens flare effect.

    Args:
        flare_settings (FlaresConf): The settings of the
    """

    if settings is not None:
        enable_lens_flare(settings.enable)
        set_flare_scale(settings.scale)
        set_flare_num_blades(settings.blades)
        set_flare_aperture_rotation(settings.aperture_rotation)
        set_flare_sensor_aspect_ratio(settings.sensor_aspect_ratio)
        set_flare_sensor_diagonal(settings.sensor_diagonal)
        set_flare_fstop(settings.fstop)
        set_flare_focal_length(settings.focal_length)


def enable_lens_flare(enable: bool = True) -> None:
    """
    Enables the lens flare effect.

    Args:
        enable (bool): True to enable the lens flare, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/enabled", enable)


def set_flare_scale(value: float = 0.0) -> None:
    """
    Sets the scale of the lens flare.

    Args:
        value (float): The scale of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/flareScale", value)


def set_flare_num_blades(value: int = 9) -> None:
    """
    Sets the number of blades of the lens flare.
    A small number will create sharp spikes, a large number will create a smooth circle.

    Args:
        value (int): The number of blades of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/blades", int(value))


def set_flare_aperture_rotation(value: float = 0.0) -> None:
    """
    Sets the rotation of the lens flare.

    Args:
        value (float): The rotation of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/apertureRotation", value)


def set_flare_sensor_diagonal(value: float = 28.0) -> None:
    """
    Sets the sensor diagonal of the lens flare.

    Args:
        value (float): The sensor diagonal of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/sensorDiagonal", value)


def set_flare_sensor_aspect_ratio(value: float = 1.5) -> None:
    """
    Sets the sensor aspect ratio of the lens flare.

    Args:
        value (float): The sensor aspect ratio of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/sensorAspectRatio", value)


def set_flare_fstop(value: float = 2.8) -> None:
    """
    Sets the f-stop of the lens flare.

    Args:
        value (float): The f-stop of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/fNumber", value)


def set_flare_focal_length(value: float = 12.0):
    """
    Sets the focal length of the lens flare.

    Args:
        value (float): The focal length of the lens flare.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/lensFlares/focalLength", value)


# ==============================================================================
# Chromatic Aberration Controls
# ==============================================================================


def set_chromatic_aberrations(cfg):
    """
    Sets the chromatic aberration settings.
    """

    if "chromatic_aberrations" in cfg["rendering"].keys():
        chromatic_aberrations_cfg = cfg["rendering"]["chromatic_aberrations"]
        apply_chromatic_aberrations(chromatic_aberrations_cfg)


def apply_chromatic_aberrations(settings: ChromaticAberrationsConf = None) -> None:
    """
    Applies the chromatic aberration effect.

    Args:
        settings (ChromaticAberrationsConf): The settings of the chromatic aberration.
    """
    if settings is not None:
        enable_chromatic_aberrations(settings.enable)
        set_chromatic_aberrations_strength(settings.strength)
        set_chromatic_aberrations_model(settings.model)
        set_chromatic_aberrations_lanczos(settings.enable_lanczos)


def enable_chromatic_aberrations(enable: bool = True) -> None:
    """
    Enables the chromatic aberration effect.

    Args:
        enable (bool): True to enable the chromatic aberration, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/enabled", enable)


def set_chromatic_aberrations_strength(value: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
    """
    Sets the strength of the chromatic aberration.

    Args:
        value (Tuple[float,float,float]): The strength of the chromatic aberration.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/strengthR", value[0])
    settings.set("/rtx/post/chromaticAberration/strengthG", value[1])
    settings.set("/rtx/post/chromaticAberration/strengthB", value[2])


def set_chromatic_aberrations_model(value: Tuple[str, str, str] = ("Radial", "Radial", "Radial")) -> None:
    """
    Sets the model of the chromatic aberration.

    Args:
        value (Tuple[str,str,str]): The model of the chromatic aberration.
    """

    for model in value:
        if model not in ["Radial", "Barrel"]:
            raise ValueError(f"Invalid chromatic aberration model: {model}")

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/modelR", value[0])
    settings.set("/rtx/post/chromaticAberration/modelG", value[1])
    settings.set("/rtx/post/chromaticAberration/modelB", value[2])


def set_chromatic_aberrations_lanczos(value: bool = False) -> None:
    """
    Sets the lanczos of the chromatic aberration.

    Args:
        value (bool): Set to True to enable lanczos, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/chromaticAberration/enableLanczos", value)


# ==============================================================================
# Motion Blur Controls
# ==============================================================================


def set_motion_blur(cfg):
    """
    Sets the motion blur settings.
    """

    if "motion_blur" in cfg["rendering"].keys():
        motion_blur_cfg = cfg["rendering"]["motion_blur"]
        apply_motion_blur(motion_blur_cfg)


def apply_motion_blur(settings: MotionBlurConf = None) -> None:
    """
    Applies the motion blur effect.

    Args:
        settings (MotionBlurConf): The settings of the motion blur.
    """

    if settings is not None:
        enable_motion_blur(settings.enable)
        set_motion_blur_diameter_fraction(settings.max_blur_diameter_fraction)
        set_motion_blur_exposure_fraction(settings.exposure_fraction)
        set_motion_blur_num_samples(settings.num_samples)


def enable_motion_blur(enable: bool = True) -> None:
    """
    Enables the motion blur effect.

    Args:
        enable (bool): True to enable the motion blur, False to disable it.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/enabled", enable)


def set_motion_blur_diameter_fraction(value: float = 0.0) -> None:
    """
    Sets the diameter fraction of the motion blur.

    Args:
        value (float): The diameter fraction of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/maxBlurDiameterFraction", value)


def set_motion_blur_exposure_fraction(value: float = 0.0) -> None:
    """
    Sets the exposure fraction of the motion blur.

    Args:
        value (float): The exposure fraction of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/exposureFraction", value)


def set_motion_blur_num_samples(value: int = 8) -> None:
    """
    Sets the number of samples of the motion blur.

    Args:
        value (int): The number of samples of the motion blur.
    """

    settings = carb.settings.get_settings()
    settings.set("/rtx/post/motionblur/numSamples", value)
