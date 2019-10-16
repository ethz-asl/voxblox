===============
Running Voxblox
===============

The easiest way to test out voxblox is to try it out on a dataset.
We have launch files for our `own dataset <http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017/>`_, the `Euroc Vicon Room datasets <http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets/>`_, and the `KITTI raw datasets <http://www.cvlibs.net/datasets/kitti//>`_ processed through `kitti_to_rosbag <https://github.com/ethz-asl/kitti_to_rosbag/>`_.

For each of these datasets, there's a launch file associated under `voxblox_ros/launch`.

The easiest way to start is to download the `cow and lady dataset <http://projects.asl.ethz.ch/datasets/doku.php?id=iros2017/>`_, edit the path to the bagfile in ``cow_and_lady_dataset.launch``, and then simply::

	roslaunch voxblox_ros cow_and_lady_dataset.launch

An alternative dataset the `basement dataset <https://projects.asl.ethz.ch/datasets/doku.php?id=basement2018/>`_ is also available. While this dataset lacks ground truth it demonstrates the capabilities of Voxblox running on Velodyne lidar data and uses ICP corrections to compensate for a drifting pose estimate. To run the dataset edit the path to the bagfile in ``basement_dataset.launch``, and then simply::

	roslaunch voxblox_ros basement_dataset.launch


If you open rviz, you should be able to see the the mesh visualized on the ``/voxblox_node/mesh`` topic of type voxblox_msgs/Mesh, in the ``world`` static frame, as shown below. One should source ``catkin_ws/devel/setup.bash`` before starting rviz, to make it recognize this topic type. 

The mesh only updates once per second (this is a setting in the launch file).

.. image:: http://i.imgur.com/nSX5Qsh.jpg
    :align: center

The rest of the commonly-used settings are parameters in the launch file. The the voxblox node page has the full list of settings.
