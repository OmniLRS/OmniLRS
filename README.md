# OmniLRS: Omniverse Lunar Robotics Simulator

<table>
  <tr>
    <td width="300"><img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/thumb/thumbnail.png" width="300"/><br/><b>V1</b>: Environment, craters, rocks, synthetic data generation, wheel traces</td>
    <td rowspan="3" valign="top" align="center"><img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/OmniLRSv3.png" width="520"/><br/><b>V3</b>: Integration with Mission Control System (Yamcs), Physics Models (power, radio, thermal)</td>
  </tr>
  <tr>
    <td width="300"><img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/Logov2.png" width="300"/><br/><b>V2</b>: Large Scale environment</td>
  </tr>
  <tr>
    <td width="300"><b>V2.5</b>: Migration to Isaac Sim 5.0</td>
  </tr>
</table>

OmniLRS is powered by NVIDIA Omniverse Isaacsim to provide visually realistic and physically accurate robotics simulation of lunar rovers.
- real-time GPU accelerated ray tracing for accurate 
- real-time GPU accelerated physics for wheel/soil interactions and rigid-body dynamics
- Base Terrain from NASA LRO Digital Elevation Models, rock and crater distributions from power laws
- Customizable and randomizable terrain generation and asset placement for synthetic data generation 
- Integration with ROS2 and more
- Multi-phyisics model framework to simulate the entire range of telemetry needed in lunar missions: Power, Radio, Thermal



## Simulation Environments Overview


|  <div style="width:70px">Name</div>  |  <div style="width:230px">Description</div>  | Images            |
|------------|-------------|---------------------------------|
| **Lunalab**            |  <div style="width:230px"> Digital-Twin of lunar analog at the University of Luxembourg. This environment also supports terrain deformation as the rover drives on it. </div> | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/lunalab.png" width=520/> |
| **Lunaryard**            |  <div style="width:230px">A small scale procedually generated lunar environment. If lunar coordinates and a date is provided the position of the earth and sun are computed using ephemerides resulting in realistic lighting. This feature is also available in the large scale environments. This environment also support terrain deformation as the rover drives on it.</div>  | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/lunaryard_husky_ex1.png" width=520/> |
| **LargeScale**           |  <div style="width:230px">Semi procedural lunar environment. It uses real DEM to reconstuct the coarse terrain, usually 5meters per pixel and then uses procedural generation to augment it to 2.5cm per pixel. The terrain itself can be generated at a even higher resolution to smooth out shadows. This very fine terrain allows to reconstruct fine terrain features increasing the engineering value of the sim. The whole of this is bundled inside Geometry clip maps, allowing to render very large scenes.</div> | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/large_scale.png" width=520/>

> [!IMPORTANT]
> This readme showcases only basic information: for a more complete introduction to the simulation and its inner workings please [visit our wiki](https://github.com/OmniLRS/OmniLRS/wiki)! 
> For specific questions or to have a chat join [our discord](https://discord.gg/KfZ2uaMHqh)! 
> Should you run into a bug, or would like to request a new feature, feel free to open an issue. 
> Want to collaborate, reach out to us! PRs very welcome!

## OmniLRS in Action!

First release:

[![First Release Demo Youtube Video](https://img.youtube.com/vi/PebUZjm0WuA/0.jpg)](https://www.youtube.com/watch?v=PebUZjm0WuA)

Wheel traces:

[![Wheel Traces Demo Youtube Video](https://img.youtube.com/vi/TpzD0h-5hv4/0.jpg)](https://www.youtube.com/watch?v=TpzD0h-5hv4)

Large Scale update:

[![Large Scale Update Demo Youtube Video](https://img.youtube.com/vi/3m78fO5uXwA/0.jpg)](https://www.youtube.com/watch?v=3m78fO5uXwA)


## Installation

Follow the instruction on the Wiki: we strongly recommend following the [docker installation](https://github.com/OmniLRS/OmniLRS/wiki/Installation#docker-installation) steps. 

Then Follow the [Getting Started steps](https://github.com/OmniLRS/OmniLRS/wiki/GettingStarted) to run your first OmniLRS simulations!

### ROS2 Integration
See the separate git repository to run the ROS2 demo. \
It supports joystick teleoperation and navigation. \
https://github.com/jnskkmhr/omnilrs_ros2_demo

The simulation allows user to interact with the Scene through ROS topics. This allows for instance to reset or teleport a robot, or to change the intensity of a light! We provide a complete description of the interactions available with the sim from ROS on the Wiki [here](https://github.com/OmniLRS/OmniLRS/wiki/ros_topics).

To use this simulation with SpaceROS, the ROS2 simulation docker must first be spinned up, and then in a second time, another container running SpaceROS must be launched to interact with the simulation.
To illustrate this, we provide a simple teleop demonstration with the sim in ROS2 and SpaceROS sending velocity commands. Check the Wiki for a [step-by-step guide](https://github.com/OmniLRS/OmniLRS/wiki#run-with-spaceros) on how to run this demo.

## Citation
Please use the following citations if you use `OmniLRS` in your work.
```bibtex
@article{richard2024omnilrs,
  title={OmniLRS: A Photorealistic Simulator for Lunar Robotics},
  author={Richard, Antoine and Kamohara, Junnosuke and Uno, Kentaro and Santra, Shreya and van der Meer, Dave and Olivares-Mendez, Miguel and Yoshida, Kazuya},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  url={https://arxiv.org/abs/2309.08997},
  year={2024}
}

@article{kamohara2024modelingterraindeformationgrouser,
      title={Modeling of Terrain Deformation by a Grouser Wheel for Lunar Rover Simulation}, 
      author={Junnosuke Kamohara and Vinicius Ares and James Hurrell and Keisuke Takehana and Antoine Richard and Shreya Santra and Kentaro Uno and Eric Rohmer and Kazuya Yoshida},
      year={2024},
      eprint={2408.13468},
      booktitle={21st International and 12th Asia-Pacific Regional Conference of the ISTVS}
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2408.13468}, 
}
```

## Directory Structure
```bash
.
├── assets
├── cfg
│   ├── environment
│   ├── mode
│   ├── physics
│   └── rendering
├── docs
├── scripts
├── src
│   ├── configurations
│   ├── environments
│   ├── environments_wrappers
│   │   ├── ros2
│   │   └── sdg
│   ├── labeling
│   ├── physics
│   ├── robots
│   ├── stellar
│   └── terrain_management
└── WorldBuilders
```
