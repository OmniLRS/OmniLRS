#!/bin/bash

# Pulls WorldBuilder
echo "Pulling WorldBuilder"
git submodule init
git submodule update

# Install GDAL
echo "Installing GDAL"
sudo apt install python3 python3-dev python3-pip 
sudo apt-get install gdal-bin libgdal-dev
version=$(gdal-config --version)

# Install Python packages for Isaac Sim
echo "Installing Python packages for Isaac Sim"
if [[ -e  ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh]]
then
    ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
    ~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh -m pip install zfpy numba
elif [[ -e ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh]]
then
    ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh -m pip install opencv-python omegaconf hydra-core skyfield gdal==$version
    ~/.local/share/ov/pkg/isaac-sim-2023.1.1/python.sh -m pip install zfpy numba
else
    echo "Could not find Isaac Sim installation"
fi

echo "Installing Python packages for default Python"
python3 -m pip --upgrade pip
python3 -m pip install gdown black gdal==$version


