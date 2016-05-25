# voxblox
![voxblox_small](https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif)

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
 * CPU-only, can be run single-threaded
 * Support for multiple different layer types
 * Serialization
 * Different ways of handling uncertainty, etc.
 * Different ways of inserting pose information about scans
 * Tight ROS integration (in voxblox_ros package)



# Usage Instructions
## How to add serialization for a new layer
The following features are implemented for the TSDF voxel type:
* Store a layer to file
* Load layer from file
* Store a subset of the blocks of a layer to file
* Load blocks from file and add to a layer

In order to support serialization for other voxel types one simply needs to:


 - [ ] Implement the following functions in ```block.cc```:

```
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
```

 - [ ] Add a type enum in ```layer.h```:

```
enum Type {
    kTsdf = 1,
    kEsdf = 2,
    kOccupancy = 3,
    kYOUR_FANCY_VOXEL_ENUM = 666
 };
```

 - [ ] Implement the following function in ```layer.cc```:

```
template <>
Layer<YOUR_FANCY_VOXEL>::Type Layer<YOUR_FANCY_VOXEL>::getType() const {
  return kYOUR_FANCY_VOXEL_ENUM;
}
```
