# voxblox
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

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper:

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Building 3D Signed Distance Fields for Planning**”, *arXiv preprint arXiv:1611.03631*, 2016.

```latex
@article{oleynikova2016voxblox,
  title={Voxblox: Building 3D Signed Distance Fields for Planning},
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Nieto, Juan and Siegwart, Roland},
  journal={arXiv preprint arXiv:1611.03631},
  year={2016}
}
```

# Installation
To install voxblox, please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
These instructions are for Ubuntu, but Voxblox will also run on OS X but you're more or less on your own there.

Then install additional system dependencies (swap indigo for kinetic as necessary):
```
sudo apt-get install python-wstool python-catkin-tools ros-indigo-cmake-modules
```

Finally, add a few other dependencies.
If you don't have a catkin workspace yet, set it up as follows:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/indigo
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

If using [**SSH keys for github**](https://help.github.com/articles/connecting-to-github-with-ssh/) (recommended):
```
cd ~/catkin_ws/src/
wstool init
wstool set catkin_simple   --git git@github.com:catkin/catkin_simple.git
wstool set eigen_catkin    --git git@github.com:ethz-asl/eigen_catkin.git
wstool set eigen_checks    --git git@github.com:ethz-asl/eigen_checks.git
wstool set gflags_catkin   --git git@github.com:ethz-asl/gflags_catkin.git
wstool set glog_catkin     --git git@github.com:ethz-asl/glog_catkin.git
wstool set minkindr        --git git@github.com:ethz-asl/minkindr.git
wstool set minkindr_ros    --git git@github.com:ethz-asl/minkindr_ros.git
wstool set voxblox         --git git@github.com:ethz-asl/voxblox.git
wstool update
```

If **not using SSH** keys but using https instead:
```
cd ~/catkin_ws/src/
wstool init
wstool set catkin_simple   --git https://github.com/catkin/catkin_simple.git
wstool set eigen_catkin    --git https://github.com/ethz-asl/eigen_catkin.git
wstool set eigen_checks    --git https://github.com/ethz-asl/eigen_checks.git
wstool set gflags_catkin   --git https://github.com/ethz-asl/gflags_catkin.git
wstool set glog_catkin     --git https://github.com/ethz-asl/glog_catkin.git
wstool set minkindr        --git https://github.com/ethz-asl/minkindr.git
wstool set minkindr_ros    --git https://github.com/ethz-asl/minkindr_ros.git
wstool set voxblox         --git https://github.com/ethz-asl/voxblox.git
wstool update
```

# Running Voxblox
The easiest way to test out voxblox is to try it out on a dataset.
We have launch files for our [own dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017), the [Euroc Vicon Room datasets](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), and the [KITTI raw datasets](http://www.cvlibs.net/datasets/kitti/) processed through [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag).

For each of these datasets, there's a launch file associated under `voxblox_ros/launch`.

The easiest way to start is to download the [cow and lady dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017), edit the path to the bagfile in `cow_and_lady_dataset.launch`, and then simply:
```roslaunch voxblox_ros cow_and_lady_dataset.launch```

If you open rviz, you should be able to see the the mesh visualized on the `/voxblox_node/mesh` MarkerArray topic, in the `world` static frame, as shown below.
The mesh only updates once per second (this is a setting in the launch file).

![cow_and_lady_rviz](https://cloud.githubusercontent.com/assets/5616392/25223468/bbd6b3cc-25bb-11e7-8c06-61baa87655ca.png)

The rest of the commonly-used settings are parameters in the launch file.

# Modifying Voxblox
Here's some hints on how to extend voxblox to fit your needs...

## Serialization

Serialization is currently implemented for:
 * TSDF layers
 * ESDF layers
 * Occupancy layers
 * ...

The following serialization tools are implemented:
* Store a layer to file
* Load layer from file
* Store a subset of the blocks of a layer to file
* Load blocks from file and add to a layer

### How to add your own voxel/layer type

- [ ] Add your own voxel type and implement the ```getVoxelType()```, e.g. **```fancy_voxel.h```** :

```cpp
namespace voxblox {

// Used for serialization only.
namespace voxel_types {
 const std::string kYOUR_FANCY_VOXEL = "fancy_voxel"
}  // namespace voxel_types

template <>
inline std::string getVoxelType<YOUR_FANCY_VOXEL>() {
 return voxel_types::kYOUR_FANCY_VOXEL;
}

}  // namespace voxblox
```

 - [ ] Implement the block (de)serialization functions for your voxel type, e.g. **```fancy_block_serialization.cc```**

```cpp
namespace voxblox {

template <>
void Block<YOUR_FANCY_VOXEL>::DeserializeVoxelData(const BlockProto& proto,
                                            YOUR_FANCY_VOXEL* voxels) {
// Your serialization code.
}

template <>
void Block<YOUR_FANCY_VOXEL>::SerializeVoxelData(const YOUR_FANCY_VOXEL* voxels,
                                          BlockProto* proto) const {
// Your serialization code.
}

}  // namespace voxblox
```

 - [ ] Create your own fancy_integrator.h, fancy_mesh_integrator.h, ...

  **Have a look at the example package:**

  TODO(mfehr, helenol): add example package with a new voxel type
