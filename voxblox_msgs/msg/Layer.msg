# Layer definitions
float64 voxel_size
uint32 voxels_per_side
uint8 layer_type  # See type enums below

# Whether to send a full map or an incremental update.
uint8 action   # See action defines below

voxblox_msgs/Block[] block

# Type definitions (should match VoxelTypes enum in voxblox/common/voxel.h)
uint8 TYPE_NOT_SERIALIZABLE = 0
uint8 TYPE_TSDF = 1
uint8 TYPE_ESDF = 2
uint8 TYPE_OCCUPANCY = 3
uint8 TYPE_LABEL = 4

# Action definitions
uint8 ACTION_FULL_MAP = 0
uint8 ACTION_UPDATE = 1
