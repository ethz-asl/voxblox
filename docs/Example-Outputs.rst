===============
Example Outputs
===============

Machine Hall
============
A mesh produced by Voxblox running inside a manifold mapper that fuses a SLAM systems poses with the output of a realsense D415 depthcamera. The map was generated while all systems were running fully onboard the pictured micro aerial vehicle.

.. image:: https://i.imgur.com/t5DHpJh.png/
  :align: center

A higher resolution mesh of the same area that was processed by voxblox offline is shown below.

.. image:: https://i.imgur.com/pvHhVsL.png/
  :align: center

Cow and Lady Dataset
====================
Voxblox running on the cow and lady dataset on a laptop equiped with an i7-4810MQ 2.80GHz CPU. In this example the system is integrating a TSDF, generating a mesh and publishing the result to RViz in real time.

.. image:: http://i.imgur.com/2wLztFm.gif/
  :align: center

Constrained Hardware
====================
Voxblox running fully onboard the Atom processor of an Intel-Euclid. Again, the system is integrating, meshing and publishing in realtime. In this example the system was also sharing the CPU with the localization system (ROVIO) and the sensor drivers. This left around one CPU core for Voxblox to use.

.. image:: http://i.imgur.com/98nAed3.gif/
  :align: center

KITTI Dataset
=============
A mesh produced from Voxblox when run on the KITTI dataset on a Desktop PC. The given localization solution and the pointcloud produced by the Velodyne were used.

.. image:: http://i.imgur.com/jAgLrZk.jpg/
  :align: center

EuRoC Dataset
=============
A voxblox mesh produced by the Maplab library running on the Stereo data provided by the EuRoC dataset.

.. image:: https://raw.githubusercontent.com/wiki/ethz-asl/maplab/readme_images/stereo.png
  :align: center

Beach Mapping
=============
A map of a beach produced by a platform with two sets of stereo cameras flying an automated coverage path.

.. image:: http://i.imgur.com/uiE7WAx.gif/
  :align: center
