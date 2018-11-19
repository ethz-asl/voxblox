=======
Voxblox
=======

.. image:: https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif
  :align: center

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:

* CPU-only, can be run single-threaded or multi-threaded for some integrators
* Support for multiple different layer types (containing different types of voxels)
* Serialization using protobufs
* Different ways of handling weighting during merging
* Different ways of inserting pose information about scans
* Tight ROS integration (in voxblox_ros package)
* Easily extensible with whatever integrators you want
* Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

.. image:: http://i.imgur.com/2wLztFm.gif
    :align: center

.. toctree::
   :maxdepth: 3
   :caption: Table of Contents
   :glob:

   pages/*

   api/library_root

Paper and Video
===============
A video showing sample output from voxblox can be seen `here <https://www.youtube.com/watch?v=PlqT5zNsvwM/>`_.
A video of voxblox being used for online planning on-board a multicopter can be seen `here <https://youtu.be/lrGSwAPzMOQ/>`_.

If using voxblox for scientific publications, please cite the following paper, available `here <http://helenol.github.io/publications/iros_2017_voxblox.pdf/>`_:

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.::

    @inproceedings{oleynikova2017voxblox,
      author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
      booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
      year={2017}
    }

Credits
=======
This library was written primarily by Helen Oleynikova and Marius Fehr, with significant contributions from Zachary Taylor, Alexander Millane, and others. The marching cubes meshing and ROS mesh generation were taken or heavily derived from `open_chisel <https://github.com/personalrobotics/OpenChisel/>`_. We've retained the copyright headers for the relevant files.

.. image:: https://i.imgur.com/pvHhVsL.png
    :align: center
