# A mesh message that support multiple instances of meshes.

# header of the MultiMesh message. This header is also used for the 'mesh' field when read by the rviz plugin.
std_msgs/Header header

# This id is similar to markers/id, where identical id's are mapped to identical visuals.
uint32 id

# sets the opacity of the entire mesh.
uint8 alpha

# The message containing the voxblox mesh.
# NOTE: if 'mesh.mesh_blocks' is empty the according visual will be cleared.
voxblox_msgs/Mesh mesh
