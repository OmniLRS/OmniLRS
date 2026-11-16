import argparse
import os

import numpy as np
from osgeo import gdal


def parse_args():
    parser = argparse.ArgumentParser(description="Preprocess DEM data")
    parser.add_argument("--dem_path", type=str, default="tmp/ldem_87s_5mpp.tif", help="Path to DEM data")
    parser.add_argument(
        "--output_dir", type=str, default="assets/Terrains/SouthPole/LDEM_87S", help="Path to save preprocessed data"
    )
    parser.add_argument("--output_name", type=str, default="dem.npy", help="Name of the output file")
    return parser.parse_args()


def preprocess_dem(dem_path: str = "", output_dir: str = "", output_name: str = "") -> None:
    """
    Preprocess DEM data

    Args:
        dem_path (str): path to DEM data
        output_dir (str): path to save preprocessed data
        output_name (str): name of the output file
    """

    ds = gdal.Open(dem_path)
    channel = np.array(ds.GetRasterBand(1).ReadAsArray())
    os.makedirs(output_dir, exist_ok=True)
    np.save(os.path.join(output_dir, output_name), channel)


if __name__ == "__main__":
    preprocess_dem(**vars(parse_args()))
