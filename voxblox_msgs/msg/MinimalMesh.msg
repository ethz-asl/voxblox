# like mesh but makes sacrifices to reduce size
# no color information
# triangle positions rounded to nearest BLOCK_EDGE_LENGTH/65535 
# triangles must be within 32,767 blocks of [0,0,0]

std_msgs/Header header

float32 block_edge_length

voxblox_msgs/MinimalMeshBlock[] mesh_blocks
