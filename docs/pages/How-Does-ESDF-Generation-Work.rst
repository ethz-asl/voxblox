==============================
How Does ESDF Generation Work?
==============================

Description of the algorithm
============================

The algorithm id described in `this paper <http://helenol.github.io/publications/iros_2017_voxblox.pdf/>`_:

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.

.. code-block:: latex

	@inproceedings{oleynikova2017voxblox,
	  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
	  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
	  year={2017}
	}


We have some system flowcharts below to make it easier to understand the general flow of data.

How TSDF values are propagated to ESDF
--------------------------------------

.. image:: https://user-images.githubusercontent.com/5616392/45752912-4dc7a780-bc17-11e8-80fe-9b5a43b373f5.png
    :align: center

How the Raise Wavefront works
-----------------------------

.. image:: https://user-images.githubusercontent.com/5616392/45752919-50c29800-bc17-11e8-9737-69929f252d85.png
    :align: center

How the Lower Wavefront works
-----------------------------

.. image:: https://user-images.githubusercontent.com/5616392/45752914-4ef8d480-bc17-11e8-93cb-4230b57eb186.png
    :align: center
