# A mesh message that support multiple instances of meshes.

# header of the MultiMesh message. This header is also used for the 'mesh' field when read by the rviz plugin.
std_msgs/Header header

# Each submap is discerned by its namespace. These can be hierarchical using '/' as a separator.
string name_space

# sets the opacity of the entire mesh.
uint8 alpha

# The message containing the voxblox mesh.
# NOTE: if 'mesh.mesh_blocks' is empty the according visual will be cleared.
voxblox_msgs/Mesh mesh
