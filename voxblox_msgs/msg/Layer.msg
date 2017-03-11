# Layer definitions
float64 voxel_size
uint32 voxels_per_side
string layer_type  # See voxblox::voxel_types

# Whether to send a full map or an incremental update.
uint8 action   # See action defines below

voxblox_msgs/Block[] blocks

# Action definitions
uint8 ACTION_FULL_MAP = 0
uint8 ACTION_UPDATE = 1
