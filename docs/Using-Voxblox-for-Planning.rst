==========================
Using Voxblox for Planning
==========================

The planners described in `Continuous-Time Trajectory Optimization for Online UAV Replanning <http://helenol.github.io/publications/iros_2016_replanning.pdf/>`_, `Safe Local Exploration for Replanning in Cluttered Unknown Environments for Micro-Aerial Vehicles <http://helenol.github.io/publications/ral_2018_local_exploration.pdf/>`_, and `Sparse 3D Topological Graphs for Micro-Aerial Vehicle Planning <https://arxiv.org/pdf/1803.04345.pdf/>`_ will be open-sourced shortly.

In the mean-time, the general idea behind using voxblox for planning is to have two nodes running: one for the mapping, which ingests pointcloud data and produces both a TSDF and an ESDF, and one for planning, which subscribes to the latest ESDF layer over ROS.

The planner should have a ``voxblox::EsdfServer`` as a member, and simply remap the ``esdf_map_out`` and ``esdf_map_in`` topics to match.

A sample launch file is shown below:

.. code-block:: xml

  <launch>
    <arg name="robot_name" default="my_robot" />
    <arg name="voxel_size" default="0.20" />
    <arg name="voxels_per_side" default="16" />
    <arg name="world_frame" default="odom" />
    <group ns="$(arg robot_name)">

      <node name="voxblox_node" pkg="voxblox_ros" type="esdf_server" output="screen" args="-alsologtostderr" clear_params="true">
        <remap from="pointcloud" to="great_sensor/my_pointcloud"/>
        <remap from="voxblox_node/esdf_map_out" to="esdf_map" />
        <param name="tsdf_voxel_size" value="$(arg voxel_size)" />
        <param name="tsdf_voxels_per_side" value="$(arg voxels_per_side)" />
        <param name="publish_esdf_map" value="true" />
        <param name="publish_pointclouds" value="true" />
        <param name="use_tf_transforms" value="true" />
        <param name="update_mesh_every_n_sec" value="1.0" />
        <param name="clear_sphere_for_planning" value="true" />
        <param name="world_frame" value="$(arg world_frame)" />
      </node>

      <node name="my_voxblox_planner" pkg="voxblox_planner" type="my_voxblox_planner" output="screen" args="-alsologtostderr">
        <remap from="odometry" to="great_estimator/odometry" />
        <remap from="my_voxblox_planner/esdf_map_in" to="esdf_map" />
        <param name="tsdf_voxel_size" value="$(arg voxel_size)" />
        <param name="tsdf_voxels_per_side" value="$(arg voxels_per_side)" />
        <param name="update_mesh_every_n_sec" value="0.0" />
        <param name="world_frame" value="$(arg world_frame)" />
      </node>

    </group>
  </launch>

And some scaffolding for writing your own planner using ESDF collision checking:

.. code-block:: c++

  class YourPlannerVoxblox {
   public:
    YourPlannerVoxblox(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);
    virtual ~YourPlannerVoxblox() {}
    double getMapDistance(const Eigen::Vector3d& position) const;
   private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Map!
    voxblox::EsdfServer voxblox_server_;
  };

There's also a traversability pointcloud you can enable/disable, that if you set the radius to your robot's collision checking radius, can show you parts of the map the planner thinks are traversable in a pointcloud:

.. code-block:: c++

  YourPlannerVoxblox::YourPlannerVoxblox(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
      : nh_(nh),
        nh_private_(nh_private),
        voxblox_server_(nh_, nh_private_) {
    // Optionally load a map saved with the save_map service call in voxblox.
    std::string input_filepath;
    nh_private_.param("voxblox_path", input_filepath, input_filepath);
    if (!input_filepath.empty()) {
      if (!voxblox_server_.loadMap(input_filepath)) {
        ROS_ERROR("Couldn't load ESDF map!");
      }
    }
    double robot_radius = 1.0;
    voxblox_server_.setTraversabilityRadius(robot_radius);
    voxblox_server_.publishTraversable();
  }


Then to check for collisions you can just compare map distance to your robot radius:

.. code-block:: c++

  double YourPlannerVoxblox::getMapDistance(
      const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
      return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position,
                                                                &distance)) {
      return 0.0;
    }
    return distance;
  }
