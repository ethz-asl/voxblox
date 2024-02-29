# Voxblox

[![Build Test](https://github.com/ethz-asl/voxblox/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/voxblox/actions/workflows/build_test.yml)

![voxblox_small](https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif)

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
 * CPU-only, can be run single-threaded or multi-threaded for some integrators
 * Support for multiple different layer types (containing different types of voxels)
 * Serialization using protobufs
 * Different ways of handling weighting during merging
 * Different ways of inserting pose information about scans
 * Tight ROS integration (in voxblox_ros package)
 * Easily extensible with whatever integrators you want
 * Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

**If you're looking for skeletonization/sparse topology or planning applications, please refer to the [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning) repo.**
**If you want to create ground truth maps from meshes or gazebo environments, please check out the [voxblox_ground_truth](https://github.com/ethz-asl/voxblox_ground_truth) pakage!**

![example_gif](http://i.imgur.com/2wLztFm.gif)

# Migration to ROS2

Voxblox has been partly migrated to ROS2 in the `ros2` git branch.
The migrated code was tested on Ubuntu 22.04 with ROS2 humble.
A Dockerfile is also available for getting started quickly. 

The code of the following Voxblox executables and their dependencies have been migrated to ROS2:

- [x] tsdf_server_node
- [ ] esdf_server_node
- [ ] intensity_server_node
- [ ] visualize_tsdf
- [ ] voxblox_eval

The following work still needs to be completed:

- [ ] Migrate remaining executables to ROS2
- [ ] Migrate tests to ROS2
- [ ] Update documentation for ROS2

# Documentation
* All voxblox documentation can be found on [our readthedocs page](https://voxblox.readthedocs.io/en/latest/index.html)

## Table of Contents
* [Paper and Video](#paper-and-video)
* [Credits](#credits)
* [Example Outputs](https://voxblox.readthedocs.io/en/latest/pages/Example-Outputs.html)
* [Performance](https://voxblox.readthedocs.io/en/latest/pages/Performance.html)
* [Installation](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)
* [Running Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html)
* [Using Voxblox for Planning](https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html)
* [Transformations in Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Transformations.html)
* [Contributing to Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Modifying-and-Contributing.html)
* [Library API](https://voxblox.readthedocs.io/en/latest/api/library_root.html)

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper, available [here](http://helenol.github.io/publications/iros_2017_voxblox.pdf):

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.

```latex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```

# Credits
This library was written primarily by Helen Oleynikova and Marius Fehr, with significant contributions from Zachary Taylor, Alexander Millane, and others. The marching cubes meshing and ROS mesh generation were taken or heavily derived from [open_chisel](https://github.com/personalrobotics/OpenChisel). We've retained the copyright headers for the relevant files.

![offline_manifold](https://i.imgur.com/pvHhVsL.png)

# Getting started

Follow these instructions to get started quickly with Voxblox for ROS2.

## Dev Container

Using an official ROS Docker container as your dev environment minimizes the amount of dependency issues and creates a consistent environment for collaboration.

To open this project in a Dev Container:

- Install Docker.
- Install VS Code.
- Install the VS Code Dev Containers extension.
- Using the VS Code extension, open the workspace in the Dev Container provided by the `.devcontainer` directory.

## Dependencies

The container specified in [.devcontainer/Dockerfile](.devcontainer/Dockerfile) contains all the required dependencies.
When the Dev Container is open, a `postCreateCommand` runs to install the rosdep dependencies.

On your host machine you can install the rosdep dependencies in this way.
Open a terminal in the project directory, and run these commands:

```
sudo rosdep init
rosdep update
rosdep install --from-paths . -y --ignore-src
```

## Build

For production, open a terminal in the project directory and run:
```
colcon build
```

For development builds use the correct flags to ensure debug symbols are built and that installation files are symlinked for quicker dev changes:

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Only rebuild the modified package for quicker builds:

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select voxblox_ros
```

## Run 

After building the source code, download some rosbag datasets to test Voxblox.
Remember to source the workspace before running `ros2 launch` or `ros2 run`.

```
source /opt/ros/humble/setup.bash
source /home/ws/install/setup.bash
```

### Download datasets

Voxblox2 was tested with:
- [VLP16 Lidar Basement dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=basement2018/)
- ["cow-and-lady" RGBD Dataset with Structure Ground Truth](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017/)

These datasets are available as ROS1 bag files. 
For use with Voxblox2 they need to be converted to ROS2 bags using the [rosbags](https://gitlab.com/ternaris/rosbags) package.

Open a terminal in the project directory and run these commands to:
- install rosbags pip package
- download the ROS1 bags of the datasets
- convert the ROS1 bags to ROS2 bags
- delete the ROS1 bags to avoid confusion

```
mkdir data
cd data

python3 -m pip install rosbags 

wget -O basement_dataset.bag http://robotics.ethz.ch/~asl-datasets/2018_basement_voxblox/basement_dataset.bag
rosbags-convert basement_dataset.bag

wget -O cow_and_lady_dataset.bag http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag
rosbags-convert cow_and_lady_dataset.bag

rm basement_dataset.bag cow_and_lady_dataset.bag
```

### Basement dataset

To run Voxblox with LiDAR data from the `basement_dataset` bag, use the following launch file with preset parameter values.
This will start the `tsdf_server_node`, `rviz2`, and play the `basement_dataset` bag.

```
ros2 launch voxblox_ros basement_dataset.launch.py
```

### RGBD cow-and-lady dataset

To run Voxblox with RGBD data from the `cow_and_lady_dataset` bag, use the following launch file with preset parameter values.
This will start the `tsdf_server_node`, `rviz2`, and play the `cow_and_lady_dataset` bag.
```
ros2 launch voxblox_ros basement_dataset.launch.py
```

### Debug 

Use the `launch.json`` configuration from VS Code and the ROS extensions to debug nodes started with a launch file.
Ensure the packages you want to debug were built with debug flags, e.g.

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

You may also have to close and reopen VS Code after building the project for the first time for the ROS extension to load correctly.