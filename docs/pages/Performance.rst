===========
Performance
===========

The Voxblox code has prioritized readability and easy extension over performance. It was also designed to operate on systems that lack a GPU. One of the main drives to create Voxblox was to create a volumetric mapping library that fit the needs of planning for robots, because of this, and unlike many TSDF libraries all possible freespace is mapped in addition to areas close to surfaces. These design decisions limit performance, however high quality real-time mapping of large enviroments is still easily acheivable. 

Integrator Performance
======================

A table of demonstrating the performance of the merged and fast integrators on the cow and lady dataset on a i7-4810MQ 2.80GHz CPU is shown below:

+------------------------------------+---------------+---------------+ 
| Rendered Mesh                      |             Setup             | 
+====================================+===============+===============+ 
|                                    | Integrator    | Merged        |
|                                    |---------------+---------------|
|                                    | Voxel size    | 20 cm         |
|                                    |---------------+---------------|
| |merged20cm|                       | TSDF          | 56 ms / scan  |
|                                    |---------------+---------------|
|                                    | Meshing       | 2 ms / scan   |
|                                    |---------------+---------------|
|                                    | Total RAM     | 49 MB         | 
+------------------------------------+---------------+---------------+ 
|                                    | Integrator    | Fast          |
|                                    |---------------+---------------|
|                                    | Voxel size    | 20 cm         |
|                                    |---------------+---------------|
| |fast20cm|                         | TSDF          | 20 ms / scan  |
|                                    |---------------+---------------|
|                                    | Meshing       | 2 ms / scan   |
|                                    |---------------+---------------|
|                                    | Total RAM     | 62 MB         | 
+------------------------------------+---------------+---------------+ 
|                                    | Integrator    | Merged        |
|                                    |---------------+---------------|
|                                    | Voxel size    | 5 cm          |
|                                    |---------------+---------------|
| |merged5cm|                        | TSDF          | 112 ms / scan |
|                                    |---------------+---------------|
|                                    | Meshing       | 10 ms / scan  |
|                                    |---------------+---------------|
|                                    | Total RAM     | 144 MB        | 
+------------------------------------+---------------+---------------+ 
|                                    | Integrator    | Fast          |
|                                    |---------------+---------------|
|                                    | Voxel size    | 5 cm          |
|                                    |---------------+---------------|
| |fast5cm|                          | TSDF          | 23 ms / scan  |
|                                    |---------------+---------------|
|                                    | Meshing       | 12 ms / scan  |
|                                    |---------------+---------------|
|                                    | Total RAM     | 153 MB        | 
+------------------------------------+---------------+---------------+ 
|                                    | Integrator    | Merged        |
|                                    |---------------+---------------|
|                                    | Voxel size    | 2 cm          |
|                                    |---------------+---------------|
| |merged2cm|                        | TSDF          | 527 ms / scan |
|                                    |---------------+---------------|
|                                    | Meshing       | 66 ms / scan  |
|                                    |---------------+---------------|
|                                    | Total RAM     | 609 MB        | 
+------------------------------------+---------------+---------------+ 
|                                    | Integrator    | Fast          |
|                                    |---------------+---------------|
|                                    | Voxel size    | 2 cm          |
|                                    |---------------+---------------|
| |fast2cm|                          | TSDF          | 63 ms / scan  |
|                                    |---------------+---------------|
|                                    | Meshing       | 110 ms / scan |
|                                    |---------------+---------------|
|                                    | Total RAM     | 673 MB        | 
+------------------------------------+---------------+---------------+ 

.. |merged20cm| image:: http://i.imgur.com/NYykPND.jpg

.. |fast20cm| image:: http://i.imgur.com/tHmGk8h.jpg

.. |merged5cm| image:: http://i.imgur.com/9KNmnum.jpg

.. |fast5cm| image:: http://i.imgur.com/3zolhrB.jpg

.. |merged2cm| image:: http://i.imgur.com/9WvpFIt.jpg

.. |fast2cm| image:: http://i.imgur.com/GcYiHZ1.jpg

Runtime Comparison to Octomap
=============================

Octomap ray casts all points from the origin, without bundling or any other approximation techniques. This is the same approach taken by voxblox's simple integrator. This leads to a significant difference in the integration times when compared to voxblox's fast integrator. A comparison in the performance was run integrating the velodyne data from the first 60 seconds of the 2011_10_03_drive_0027_sync KITTI dataset. The test was performed on an Intel i7-4810MQ quad core CPU running at 2.8 GHz and truncation distance was set to 4 voxels. The results are shown in the table below:

+-----------------+----------------+--------------------------+--------------------------+ 
| Voxel size      | Max ray length | Octomap integration time | Voxblox integration time |
+=================+================+==========================+==========================+ 
|0.5 m            | 10 m           | 38 ms / scan             | 14 ms / scan             |
+-----------------+----------------+--------------------------+--------------------------+
|0.5 m            | 50 m           | 63 ms / scan             | 14 ms / scan             |
+-----------------+----------------+--------------------------+--------------------------+
|0.2 m            | 10 m           | 136 ms / scan            | 18 ms / scan             |
+-----------------+----------------+--------------------------+--------------------------+
|0.2 m            | 50 m           | 760 ms / scan            | 44 ms / scan             |
+-----------------+----------------+--------------------------+--------------------------+
|0.1 m            | 10 m           | 905 ms / scan            | 35 ms / scan             |
+-----------------+----------------+--------------------------+--------------------------+
|0.1 m            | 50 m           | 3748 ms / scan           | 100 ms / scan            |
+-----------------+----------------+--------------------------+--------------------------+

On the same dataset voxblox was found to use 2 to 3 times the ram of Octomap.
