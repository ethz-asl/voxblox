=======================
Contributing to Voxblox
=======================

These steps are only necessary if you plan on contributing to voxblox.

Code style
==========

We follow the style and best practices listed in the `Google C++ Style Guide <https://google.github.io/styleguide/cppguide.html/>`_.

Setting up the linter
---------------------

This sets up a linter which checks if the code conforms to our style guide during commits.

First, install the dependencies listed `here <https://github.com/ethz-asl/linter#dependencies/>`_.

.. code-block:: bash

	cd ~/catkin_ws/src/
	git clone git@github.com:ethz-asl/linter.git
	cd linter
	echo ". $(realpath setup_linter.sh)" >> ~/.bashrc  # Or the matching file for
	                                                   # your shell.
	bash

	# Initialize linter in voxblox repo
	cd ~/catkin_ws/src/voxblox
	init_linter_git_hooks

For more information about the linter visit `ethz/linter <https://github.com/ethz-asl/linter/>`_

Modifying Voxblox
=================
Here's some hints on how to extend voxblox to fit your needs...

Serialization
-------------

Serialization is currently implemented for:

* TSDF layers
* ESDF layers
* Occupancy layers

The following serialization tools are implemented:

* Store a layer to file
* Load layer from file
* Store a subset of the blocks of a layer to file
* Load blocks from file and add to a layer

How to add your own voxel/layer type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Add your own voxel type and implement the ``getVoxelType()``, e.g. ``fancy_voxel.h`` :

.. code-block:: bash

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

- Implement the block (de)serialization functions for your voxel type, e.g. ``fancy_block_serialization.cc``

.. code-block:: bash

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

- Create your own fancy_integrator.h, fancy_mesh_integrator.h, ...

  **Have a look at the example package:**

  TODO(mfehr, helenol): add example package with a new voxel type
