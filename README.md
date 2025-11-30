<center>
<img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/Logov2.png" width=520/>
</center>

Omniverse Lunar Robotics Simulator or OmniLRS, is a simulation tools developped jointly by the Space Robotics group from the University of Luxembourg (SpaceR), and the Space Robotics Lab from Tohoku University in Japan (SRL). We are now opening it to the community and strongly encourage Space Roboticists to help us grow the feature set of this simulation! Don't be shy shoot a PR!

> [!IMPORTANT]
> This readme provides basic information on how to use the simulation. For a more complete introduction to the simulation and its inner workings please [visit our wiki](https://github.com/OmniLRS/OmniLRS/wiki)! For specific questions or to have a chat join [our discord](https://discord.gg/KfZ2uaMHqh)!

## Simulation Environments Overview


|  <div style="width:70px">Name</div>  |  <div style="width:230px">Description</div>  | Images            |
|------------|-------------|---------------------------------|
| **Lunalab**            |  <div style="width:230px"> Digital-Twin of lunar analog at the University of Luxembourg. This environment also supports terrain deformation as the rover drives on it. </div> | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/lunalab.png" width=520/> |
| **Lunaryard**            |  <div style="width:230px">A small scale procedually generated lunar environment. If lunar coordinates and a date is provided the position of the earth and sun are computed using ephemerides resulting in realistic lighting. This feature is also available in the large scale environments. This environment also support terrain deformation as the rover drives on it.</div>  | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/lunaryard_husky_ex1.png" width=520/> |
| **LargeScale**           |  <div style="width:230px">Semi procedural lunar environment. It uses real DEM to reconstuct the coarse terrain, usually 5meters per pixel and then uses procedural generation to augment it to 2.5cm per pixel. The terrain itself can be generated at a even higher resolution to smooth out shadows. This very fine terrain allows to reconstruct fine terrain features increasing the engineering value of the sim. The whole of this is bundled inside Geometry clip maps, allowing to render very large scenes.</div> | <img src="https://raw.githubusercontent.com/wiki/OmniLRS/OmniLRS/media/env_img/large_scale.png" width=520/>

> [!NOTE]
> Please note that this is a partial release. More robots will be made available at a later date. Should you run into a bug, or would like to request a new feature, feel free to open an issue. Want to collaborate, reach out to us!

## OmniLRS in Action!

First release:

[![First Release Demo Youtube Video](https://img.youtube.com/vi/PebUZjm0WuA/0.jpg)](https://www.youtube.com/watch?v=PebUZjm0WuA)

Wheel traces:

[![Wheel Traces Demo Youtube Video](https://img.youtube.com/vi/TpzD0h-5hv4/0.jpg)](https://www.youtube.com/watch?v=TpzD0h-5hv4)

Large Scale update:

[![Large Scale Update Demo Youtube Video](https://img.youtube.com/vi/3m78fO5uXwA/0.jpg)](https://www.youtube.com/watch?v=3m78fO5uXwA)


## Installation

Follow the instruction on the Wiki: we strongly recommend following the [docker installation](https://github.com/OmniLRS/OmniLRS/wiki/Installation#docker-installation) steps. 


## Getting Started:

Follow the [steps in the Wiki](https://github.com/OmniLRS/OmniLRS/wiki/GettingStarted) to run your first OmniLRS simulations!

### ROS Interactions with the Scene

The simulation allows user to interact with the Scene through ROS topics. This allows for instance to reset or teleport a robot, or to change the intensity of a light! We provide a complete description of the interactions available with the sim from ROS on the Wiki [here](https://github.com/OmniLRS/OmniLRS/wiki/ros_topics).

### ROS2 Demo
We've prepared another separate git repository to run ROS2 demo. \
It supports joystick teleoperation and navigation for now. \
https://github.com/jnskkmhr/omnilrs_ros2_demo

### Integration & Workflow with SpaceROS

Isaac Sim is using ROS2 by default, most of the tools available in Isaac are meant for ROS2. Hence, this simulation uses ROS2. To use this simulation with SpaceROS, the ROS2 simulation docker must first be spinned up, and then in a second time, another container running SpaceROS must be launched to interact with the simulation.

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
│   │   ├── ros1
│   │   ├── ros2
│   │   └── sdg
│   ├── labeling
│   ├── physics
│   ├── robots
│   ├── stellar
│   └── terrain_management
└── WorldBuilders
```
