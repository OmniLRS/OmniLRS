import argparse
import os
from typing import Tuple

import yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Process DEM data")
    parser.add_argument(
        "--info_path",
        type=str,
        default="tmp/ldem_87s_5mpp.info",
        help="Path to DEM info",
    )
    parser.add_argument("--output_dir", type=str, default="tmp", help="Path to save preprocessed info")
    parser.add_argument(
        "--output_name",
        type=str,
        default="ldem_87s_5mpp",
        help="Name of the output file",
    )
    return parser.parse_args()


def get_center(line: str) -> Tuple[float, float]:
    """
    Get the center coordinates in degrees of the DEM

    Args:
        line (str): line from the info file

    Returns:
        x_center (float): x-coordinate of the center in degrees
        y_center (float): y-coordinate of the center in degrees
    """

    # Remove leading and trailing whitespaces
    tmp = line.strip(" ")
    tmp = tmp.split("(")
    # Split the string by comma
    tmp = tmp[2].split(",")
    # Get the first and second part of the split string
    tmp1 = tmp[0]
    tmp2 = tmp[1].strip(")")

    # Extract the degrees, minutes, and seconds from the first part
    tmp1 = tmp1.split("d")
    tmp1d = tmp1[0]
    tmp1m = tmp1[1].split("'")[0]
    tmp1_sign = 1  # if the direction is East, the sign is positive
    if tmp1[1].split("'")[1].strip('"')[-1] == "E":
        tmp1s = tmp1[1].split("'")[1].strip('"E')
    else:
        tmp1s = tmp1[1].split("'")[1].strip('"W')
        tmp1_sign = -1  # if the direction is West, the sign is negative

    # Extract the degrees, minutes, and seconds from the second part
    tmp2 = tmp2.split("d")
    tmp2d = tmp2[0]
    tmp2m = tmp2[1].split("'")[0]
    tmp2_sign = 1  # if the direction is North, the sign is positive
    if tmp2[1].split("'")[1].strip('"')[-1] == "N":
        tmp2s = tmp2[1].split("'")[1].strip('"N')
    else:
        tmp2s = tmp2[1].split("'")[1].strip('"S')
        tmp2_sign = -1  # if the direction is South, the sign is negative

    # Calculate the center coordinates
    x_center = (float(tmp1d) + float(tmp1m) / 60 + float(tmp1s) / 3600) * tmp1_sign
    y_center = (float(tmp2d) + float(tmp2m) / 60 + float(tmp2s) / 3600) * tmp2_sign

    return x_center, y_center


def get_pixel_size(line: str) -> Tuple[float, float]:
    """
    Get the pixel size in meters of the DEM.

    Args:
        line (str): line from the info file

    Returns:
        x_pixel_size (float): pixel size in the x-direction in meters
        y_pixel_size (float): pixel size in the y-direction in meters
    """

    tmp = line.strip(" ")
    tmp = tmp.split("(")[1].split(")")[0].split(",")
    x_pixel_size = float(tmp[0])
    y_pixel_size = float(tmp[1])
    return x_pixel_size, y_pixel_size


def get_size(line: str) -> Tuple[int, int]:
    """
    Get the size of the DEM in pixels.

    Args:
        line (str): line from the info file

    Returns:
        x_size (int): size of the DEM in the x-direction in pixels
        y_size (int): size of the DEM in the y-direction in pixels
    """

    tmp = line.split(" ")
    x_size = int(tmp[2].strip(","))
    y_size = int(tmp[3])
    return x_size, y_size


def process_info(info_path: str = "", output_dir: str = "", output_name: str = "") -> None:
    """
    Process the DEM info file and save the processed info as a YAML file.

    Args:
        info_path (str): path to the DEM info file
        output_dir (str): path to save the processed info
        output_name (str): name of the output file
    """

    with open(info_path, "r") as f:
        info = f.readlines()
    info = [line.strip() for line in info]

    keywords = ["Center", "Pixel Size", "Size is"]

    info_dict = {}
    for keyword in keywords:
        for line in info:
            if keyword in line:
                if keyword == "Size is":
                    v = get_size(line)
                    info_dict["size"] = v
                elif keyword == "Pixel Size":
                    v = get_pixel_size(line)
                    info_dict["pixel_size"] = v
                elif keyword == "Center":
                    v = get_center(line)
                    info_dict["center_coordinates"] = v

    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, output_name + ".yaml")
    with open(output_path, "w") as f:
        yaml.safe_dump(info_dict, f)


if __name__ == "__main__":
    process_info(**vars(parse_args()))
