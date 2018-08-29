# Voxblox

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

![example_gif](http://i.imgur.com/2wLztFm.gif)

# Table of Contents
* [Paper and Video](#paper-and-video)
* [Credits](#credits)
* [Example Outputs](https://github.com/ethz-asl/voxblox/wiki/Example-Outputs)
* [Performance](https://github.com/ethz-asl/voxblox/wiki/Performance)
* [Installation](https://github.com/ethz-asl/voxblox/wiki/Installation)
* [Running Voxblox](https://github.com/ethz-asl/voxblox/wiki/Running-Voxblox)
* [Voxblox Node (TSDF Server, ESDF Server)](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node)
  * [Published and Subscribed Topics](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#published-and-subscribed-topics)
  * [Services](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#services)
  * [Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node#parameters)
    * [General Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#general-parameters)
    * [TSDF Integrator Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#tsdf-integrator-parameters)
    * [Fast TSDF Integrator Specific Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#fast-tsdf-integrator-specific-parameters)
    * [ESDF Integrator Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#esdf-integrator-parameters)
    * [ICP Refinement Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#icp-refinement-parameters)
    * [Input Transform Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#input-transform-parameters)
    * [Output Parameters](https://github.com/ethz-asl/voxblox/wiki/The-Voxblox-Node/#output-parameters)
* [Using Voxblox for Planning](https://github.com/ethz-asl/voxblox/wiki/Using-Voxblox-for-Planning)
* [Transformations in Voxblox](https://github.com/ethz-asl/voxblox/wiki/Transformations)
* [Contributing to Voxblox](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#contributing-to-voxblox)
    * [Code style](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#code-style)
    * [Setting up the linter](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#setting-up-the-linter)
* [Modifying Voxblox](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#modifying-voxblox)
  * [Serialization](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#serialization)
    * [How to add your own voxel/layer type](https://github.com/ethz-asl/voxblox/wiki/Modifying-And-Contributing/#how-to-add-your-own-voxel-layer-type)

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
