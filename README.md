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

# Table of Contents
* [Paper and Video](README.md#paper-and-video)
* [Credits](README.md#credits)
* [Example Outputs](README.md#example-outputs)
* [Performance](README.md#performance)
* [Installation](README.md#installation)
* [Running Voxblox](README.md#running-voxblox)
* [Voxblox Node](README.md#voxblox-node)
  * [Published and Subscribed Topics](README.md#published-and-subscribed-topics)
  * [Services](README.md#Services)
  * [Parameters](README.md#parameters)
* [Modifying Voxblox](README.md#modifying-voxblox)
  * [Serialization](README.md#serialization)
* [Transformations in Voxblox](README.md#transformations-in-voxblox)

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper, available [here](http://helenol.github.io/publications/iros_2017_voxblox.pdf):

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2016.

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

# Example Outputs
A mesh produced by Voxblox running inside a manifold mapper that fuses a SLAM systems poses with the output of a realsense D415 depthcamera. The map was generated while all systems were running fully onboard the pictured micro aerial vehicle.
![manifold_mapping](https://i.imgur.com/t5DHpJh.png)

A higher resolution mesh of the same area that was processed by voxblox offline is shown below.
![offline_manifold](https://i.imgur.com/pvHhVsL.png)

Voxblox running on the cow and lady dataset on a laptop equiped with an i7-4810MQ 2.80GHz CPU. In this example the system is integrating a TSDF, generating a mesh and publishing the result to RViz in real time.
![example_gif](http://i.imgur.com/2wLztFm.gif)

Voxblox running fully onboard the Atom processor of an Intel-Euclid. Again, the system is integrating, meshing and publishing in realtime. In this example the system was also sharing the CPU with the localization system (ROVIO) and the sensor drivers. This left around one CPU core for Voxblox to use.
<p align="center">
<img src="https://i.imgur.com/98nAed3.gif">
</p>

A mesh produced from Voxblox when run on the KITTI dataset on a Desktop PC. The given localization solution and the pointcloud produced by the Velodyne were used.
![velodyne_kitti](https://i.imgur.com/jAgLrZk.jpg)

A voxblox mesh produced by the Maplab library running on the Stereo data provided by the EuRoC dataset.
<p align="center">
<img src="https://raw.githubusercontent.com/wiki/ethz-asl/maplab/readme_images/stereo.png">
</p>

A map of a beach produced by a platform with two sets of stereo cameras flying an automated coverage path.
<p align="center">
<img src="https://i.imgur.com/uiE7WAx.gif">
</p>

# Performance
The Voxblox code has prioritized readability and easy extension over performance. It was also designed to operate on systems that lack a GPU. One of the main drives to create Voxblox was to create a volumetric mapping library that fit the needs of planning for robots, because of this, and unlike many TSDF libraries all possible freespace is mapped in addition to areas close to surfaces. These design decisions limit performance, however high quality real-time mapping of large enviroments is still easily acheivable. A table of the performance on the cow and lady dataset on a i7-4810MQ 2.80GHz CPU is also shown.

<center>

Rendered Mesh | Setup
--- | ---
<img src="http://i.imgur.com/NYykPND.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Merged</td></tr><tr><td>Voxel:</td><td>20 cm</td></tr><tr><td>TSDF:</td><td>56 ms / scan</td></tr><tr><td>Meshing:</td><td>2 ms / scan</td></tr><tr><td>Total RAM:</td><td>49 MB</td></tr></table>
<img src="http://i.imgur.com/tHmGk8h.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Fast</td></tr><tr><td>Voxel:</td><td>20 cm</td></tr><tr><td>TSDF:</td><td>20 ms / scan</td></tr><tr><td>Meshing:</td><td>2 ms / scan</td></tr><tr><td>Total RAM:</td><td>62 MB</td></tr></table>
<img src="http://i.imgur.com/9KNmnum.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Merged</td></tr><tr><td>Voxel:</td><td>5 cm</td></tr><tr><td>TSDF:</td><td>112 ms / scan</td></tr><tr><td>Meshing:</td><td>10 ms / scan</td></tr><tr><td>Total RAM:</td><td>144.2 MB</td></tr></table>
<img src="http://i.imgur.com/3zolhrB.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Fast</td></tr><tr><td>Voxel:</td><td>5 cm</td></tr><tr><td>TSDF:</td><td>23 ms / scan</td></tr><tr><td>Meshing:</td><td>12 ms / scan</td></tr><tr><td>Total RAM:</td><td>153.9 MB</td></tr></table>
<img src="http://i.imgur.com/9WvpFIt.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Merged</td></tr><tr><td>Voxel:</td><td>2 cm</td></tr><tr><td>TSDF:</td><td>527 ms / scan</td></tr><tr><td>Meshing:</td><td>66 ms / scan</td></tr><tr><td>Total RAM:</td><td>609.2 MB</td></tr></table>
<img src="http://i.imgur.com/GcYiHZ1.jpg" width="500"> | <table><tr><td>Integrator:</td><td>Fast</td></tr><tr><td>Voxel:</td><td>2 cm</td></tr><tr><td>TSDF:</td><td>63 ms / scan</td></tr><tr><td>Meshing:</td><td>110 ms / scan</td></tr><tr><td>Total RAM:</td><td>673.1 MB</td></tr></table>

</center>

# Installation
To install voxblox, please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).
These instructions are for Ubuntu, Voxblox will also run on OS X, but you're more or less on your own there.

First install additional system dependencies (swap kinetic for indigo or melodic as necessary):
```
sudo apt-get install python-wstool python-catkin-tools ros-kinetic-cmake-modules protobuf-compiler autoconf
```

Next, add a few other dependencies.
If you don't have a catkin workspace yet, set it up as follows:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

If using [**SSH keys for github**](https://help.github.com/articles/connecting-to-github-with-ssh/) (recommended):
```
cd ~/catkin_ws/src/
wstool init . ./voxblox/voxblox_ssh.rosinstall
wstool update
```

If **not using SSH** keys but using https instead:
```
cd ~/catkin_ws/src/
wstool init . ./voxblox/voxblox_https.rosinstall
wstool update
```

If you have already initalized wstool replace the above `wstool init` with `wstool merge -t`

Compile:
```
cd ~/catkin_ws/src/
catkin build voxblox_ros
```

# Contributing to voxblox
These steps are only necessary if you plan on contributing to voxblox.

### Code style

We follow the style and best practices listed in the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

### Setting up the linter
This setups a linter which checks if the code conforms to our style guide during commits.

First, install the dependencies listed [here](https://github.com/ethz-asl/linter#dependencies).

```bash
cd ~/catkin_ws/src/
git clone git@github.com:ethz-asl/linter.git
cd linter
echo ". $(realpath setup_linter.sh)" >> ~/.bashrc  # Or the matching file for
                                                   # your shell.
bash

# Initialize linter in voxblox repo
cd ~/catkin_ws/src/voxblox
init_linter_git_hooks
```
For more information about the linter visit  [ethz/linter](https://github.com/ethz-asl/linter)


# Running Voxblox
The easiest way to test out voxblox is to try it out on a dataset.
We have launch files for our [own dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017), the [Euroc Vicon Room datasets](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets), and the [KITTI raw datasets](http://www.cvlibs.net/datasets/kitti/) processed through [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag).

For each of these datasets, there's a launch file associated under `voxblox_ros/launch`.

The easiest way to start is to download the [cow and lady dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017), edit the path to the bagfile in `cow_and_lady_dataset.launch`, and then simply:
```roslaunch voxblox_ros cow_and_lady_dataset.launch```

An alternative dataset the [basement dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=basement2018) is also avaliable. While this dataset lacks groundtruth it demonstrates the capabilities of Voxblox running on Velodyne lidar data and uses ICP corrections to compensate for a drifting pose estimate. To run the dataset edit the path to the bagfile in `basement_dataset.launch`, and then simply:
```roslaunch voxblox_ros basement_dataset.launch```

If you open rviz, you should be able to see the the mesh visualized on the `/voxblox_node/mesh` MarkerArray topic, in the `world` static frame, as shown below.
The mesh only updates once per second (this is a setting in the launch file).

![cow_and_lady_rviz](https://i.imgur.com/nSX5Qsh.jpg)

The rest of the commonly-used settings are parameters in the launch file.

# Voxblox Node (TSDF Server, ESDF Server)

## Published and Subscribed Topics

Note: the voxblox_node has been replaced with tsdf_server (if you want a TSDF) or esdf_server (if you want both a TSDF and an ESDF).
The tsdf_server and esdf_server publish and subscribe to the following topics:

- Published topics:
  - **`mesh`** of type `voxblox_msgs::MeshBlock`. A visualization topic showing the mesh produced from the tsdf in a form that can be seen in RViz. Set `update_mesh_every_n_sec` to control its update rate.
  - **`surface_pointcloud`** of type `pcl::PointCloud<pcl::PointXYZRGB>`. A colored pointcloud of the voxels that are close to a surface.
  - **`tsdf_pointcloud`** of type `pcl::PointCloud<pcl::PointXYZI>`. A pointcloud showing all allocated voxels.
  - **`mesh_pointcloud`** of type `pcl::PointCloud<pcl::PointXYZRGB>`. Only appears if `output_mesh_as_pointcloud` is true, outputs a pointcloud containing the verticies of the generated mesh.
  - **`mesh_pcl`** of type `pcl_msgs::PolygonMesh`. Only appears if `output_mesh_as_pcl_mesh` is true, outputs any mesh generated by the generate_mesh service.
  - **`tsdf_slice`** of type `pcl::PointCloud<pcl::PointXYZI>`. Outputs a 2D horizontal slice of the TSDF colored by the stored distance value.
  - **`esdf_pointcloud`** of type `pcl::PointCloud<pcl::PointXYZI>`. A pointcloud showing the values of all allocated ESDF voxels. Only appears if using `esdf_server`.
  - **`esdf_slice`** of type `pcl::PointCloud<pcl::PointXYZI>`. Outputs a 2D horizontal slice of the ESDF colored by the stored distance value. Only appears if using `esdf_server`.
  - **`occupied_nodes`** of type `visualization_msgs::MarkerArray`. Visualizes the location of the allocated voxels in the TSDF.
  - **`tsdf_map_out`** of type `voxblox_msgs::Layer`. Publishes the entire TSDF layer to update other nodes (that listen on tsdf_layer_in). Only published if `publish_tsdf_map` is set to true.
  - **`esdf_map_out`** of type `voxblox_msgs::Layer`. Publishes the entire ESDF layer to update other nodes (that listen on esdf_layer_in). Only published if `publish_esdf_map` is set to true.

- Subscribed topics:
  - **`transform`** of type `geometry_msgs::TransformStamped`. Only appears if `use_tf_transforms` is false. The transformation from the world frame to the current sensor frame.
  - **`pointcloud`** of type `sensor_msgs::PointCloud2`. The input pointcloud to be integrated.
  - **`freespace_pointcloud`** of type `sensor_msgs::PointCLoud2`. Only appears if `use_freespace_pointcloud` is true. Unlike the `pointcloud` topic where the given points lie on surfaces, the points in the `freespace_pointcloud` are taken to be floating in empty space. These points can assist in generating more complete freespace information in a map.
  - **`tsdf_map_in`** of type `voxblox_msgs::Layer`. Replaces the current TSDF layer with that from this topic. Voxel size and voxels per side should match.
  - **`esdf_map_in`** of type `voxblox_msgs::Layer`. Replaces the current ESDF layer with that from this topic. Voxel size and voxels per side should match.
  - **`icp_transform`** of type `geometry_msgs::TransformStamped`. If ICP is enabled, this is the current corrected transform between the world frame and the ICP frame.

## Services

The tsdf_server and esdf_server have the following services:

  - **`generate_mesh`** This service has an empty request and response. Calling this service will generate a new mesh. The mesh will be saved as a ply file unless `mesh_filename` is set to "". The mesh will also be output on the `mesh_pointcloud` topic if `output_mesh_as_pointcloud` is true and on the `mesh_pcl` topic if `output_mesh_as_pcl_mesh` is true.
  - **`generate_esdf`** This service has an empty request and response. It can be used to trigger an esdf map update.
  - **`save_map`** This service has a `voxblox_msgs::FilePath::Request` and `voxblox_msgs::FilePath::Response`. The service call saves the tsdf layer to a .vxblx file.
  - **`load_map`** This service has a `voxblox_msgs::FilePath::Request` and `voxblox_msgs::FilePath::Response`. The service call loads the tsdf layer from a .vxblx file.
  - **`publish_map`** This service has an empty request and response. Publishes any TSDF and ESDF layers on the `tsdf_map_out` and `esdf_map_out` topics.
  - **`publish_pointclouds`** This service has an empty request and response. Publishes TSDF and ESDF pointclouds and slices.


## Parameters
------
A summary of the user setable tsdf_server and esdf_server parameters:

### General Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `min_time_between_msgs_sec` |  Minimum time to wait after integrating a message before accepting a new one. | 0.0 |
| `pointcloud_queue_size` | The size of the queue used to subscribe to pointclouds. | 1 |
| `verbose` | Prints additional debug and timing information. | true |
| `max_block_distance_from_body` | Blocks that are more than this distance from the latest robot pose are deleted, saving memory | 3.40282e+38 |


### TSDF Integrator Parameters

The most important parameter here is the selection of the method:
* "simple" - the most straightfoward integrator. Every point in the pointcloud has a ray cast from the origin through it. Every voxel each ray passes through is updated individually. A very slow and exact approach.
* "merged" - Rays that start and finish in the same voxel are bundled into a single ray. The properties of the points are merged and their weights added so no information is lost. The approximation means some voxels will recive updates that were otherwise meant for neighboring voxels. This approach works well with large voxels (10 cm or greater) and can give an order of magnitude speed up over the simple integrator.
* "fast" - Rays that attempt to update voxels already updated by other rays from the same pointcloud are terminated early and discarded. An approximate method that has been designed to give the fastest possible results at the expense of discarding large quantities of information. The trade off between speed and information loss can be tuned via the `start_voxel_subsampling_factor` and `max_consecutive_ray_collisions` parameters. This method is currently the only viable integrator for real-time applications with voxels smaller than 5 cm.

| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `method` | Method to use for integrating new readings into the tsdf. Options are "merged", "simple", "merged_discard" and "fast" | "merged" |
| `tsdf_voxel_size` | The size of the tsdf voxels | 0.2 |
| `tsdf_voxels_per_side` | TSDF voxels per side of an allocated block. Must be a power of 2 | 16 |
| `voxel_carving_enabled` | If true, the entire length of a ray is integrated, if false only the region inside the trunaction distance is used. | true |
| `truncation_distance` | The truncation distance for the TSDF | 2`tsdf_voxel_size` |
| `max_ray_length_m` | The maximum range out to which a ray will be cast | 5.0 |
| `min_ray_length_m` | The point at which the ray casting will start | 0.1 |
| `max_weight` | The upper limit for the weight assigned to a voxel | 10000.0 |
| `use_const_weight` | If true all points along a ray have equal weighting | false |
| `allow_clear` | If true points beyond the `max_ray_length_m` will be integrated up to this distance | true |
| `use_freespace_pointcloud` | If true a second subscription topic `freespace_pointcloud` appears. Clearing rays are cast from beyond this topic's points' truncation distance to assist in clearing freespace voxels | false |

### Fast TSDF Integrator Specific Parameters
These parameters are only used if the integrator `method` is set to "fast".

| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `start_voxel_subsampling_factor` | Before integration points are inserted into a sub-voxel, only one point is allowed per sub-voxel. This can be thought of as subsampling the pointcloud. The edge length of the sub-voxel is the voxel edge length divided by `start_voxel_subsampling_factor`. | 2 |
| `max_consecutive_ray_collisions` | When a ray is cast by this integrator it detects if any other ray has already passed through the current voxel this scan. If it passes through more than `max_consecutive_ray_collisions` voxels other rays have seen in a row, it is taken to be adding no new information and the casting stops  | 2 |
| `max_integration_time_s` | The time budget for frame integration, if this time is exceeded ray casting is stopped early. Used to guarantee real time performance. | 3.40282e+38 |
| `clear_checks_every_n_frames` | Governs how often the sets that indicate if a sub-voxel is full or a voxel has had a ray passed through it are cleared. | 1 |

### ESDF Integrator Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `generate_esdf` |  If the eucliden signed distance field should be generated. | false |
| `esdf_max_distance_m` | The maximum distance that the esdf will be calculated out to | 2.0 |
| `esdf_default_distance_m` | Default distance set for unknown values and values >`esdf_max_distance_m` | 2.0 |

### ICP Refinement Parameters
ICP based refinement can be applied to the poses of the input pointclouds before merging.

| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `enable_icp` | Whether to use ICP to align all incoming pointclouds to the existing structure. | false |
| `icp_refine_roll_pitch` | True to apply 6-dof pose correction, false for 4-dof (x, y, z, yaw) correction. | false |
| `accumulate_icp_corrections` | Whether to accumulate transform corrections from ICP over all pointclouds. Reset at each new pointcloud if false. | true |
| `icp_corrected_frame` | TF frame to output the ICP corrections to.| `icp_corrected` |
| `pose_corrected_frame` | TF frame used to output the ICP corrected poses relative to the `icp_corrected_frame`.| `pose_corrected` |
| `icp_iterations` | Number of ICP iterations to perform. | 20 |
| `icp_subsample_keep_ratio` | Random subsampling will be used to reduce the number of points used for matching by this factor.  | 0.05 |
| `icp_min_match_ratio` | For an ICP refinement to be accepted, at least this ratio of points in the pointcloud must fall within the truncation distance of the existing TSDF layer | 0.5 |

### Input Transform Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `use_tf_transforms` | If true the ros tf tree will be used to get the pose of the sensor relative to the world (`sensor_frame` and `world_frame` will be used). If false the pose must be given via the `transform` topic. | true |
| `world_frame` | The base frame used when looking up tf transforms. This is also the frame that most outputs are given in. | "world" |
| `sensor_frame` | The sensor frame used when looking up tf transforms. If set to "" the frame of the input pointcloud message will be used.  | "" |
| `T_B_D` | A static transformation from the base to the dynamic system that will be applied | N/A |
| `invert_T_B_D` | If the given `T_B_D` should be inverted before it is used | false |
| `T_B_C` | A static transformation from the base to the sensor that will be applied | N/A |
| `invert_T_B_C` | If the given `T_B_C` should be inverted before it is used | false |

### Output Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `output_mesh_as_pointcloud` | If true the verticies of the generated mesh will be ouput as a pointcloud on the topic `mesh_pointcloud` whenever the generate_mesh service is called. | false |
| `output_mesh_as_pcl_mesh` | If true the generated mesh will be ouput as a pcl::PolygonMesh on the topic `mesh_pcl` whenever the generate_mesh service is called. | false |
| `slice_level` | The height at which generated tsdf and esdf slices will be made. | 0.5 |
| `color_ptcloud_by_weight` | If the pointcloud should be colored by the voxel weighting | false |
| `mesh_filename` | Filename output mesh will be saved to, leave blank if no file should be generated | "" |
| `color_mode` | The method that will be used for coloring the mesh. Options are "color", "height", "normals", "lambert" and "gray". | "color" |
| `mesh_min_weight` | The minimum weighting needed for a point to be included in the mesh | 1e-4 |
| `update_mesh_every_n_sec` | Rate at which the mesh topic will be published to, a value of 0 disables. Note, this will not trigger any other mesh operations, such as generating a ply file. | 0.0 |
| `publish_tsdf_map` | Whether to publish the complete TSDF map periodically over ROS topics. | false |
| `publish_esdf_map` | Whether to publish the complete ESDF map periodically over ROS topics. | false |
| `publish_tsdf_info` | Enables publishing of `tsdf_pointcloud`, `surface_pointcloud` and `occupied_nodes`. | false |
| `publish_pointclouds` | If true the tsdf and esdf (if generated) is published as a pointcloud when the mesh is updated | false |
| `intensity_colormap` | If the incoming pointcloud is an intensity (not RGB) pointcloud, such as from laser, this sets how the intensities will be mapped to a color. Valid options are `rainbow`, `inverse_rainbow`, `grayscale`, `inverse_grayscale`, `ironbow` (thermal) | `rainbow` |
| `intensity_max_value` | Maximum value to use for the intensity mapping. Minimum value is always 0. | 100.0 |

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

# Transformations in Voxblox

Voxblox uses active transforms and Hamilton quaternions. For futher details on the notation used throughout the code see [the minkindr wiki](https://github.com/ethz-asl/minkindr/wiki/Common-Transformation-Conventions)
