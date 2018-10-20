# like MeshBlock but makes sacrifices to reduce size
# no color information
# triangle positions rounded to nearest BLOCK_EDGE_LENGTH/65535 
# triangles must be within 32,767 blocks of [0,0,0]

# Index of meshed points in block map
int16[3] index

voxblox_msgs/MinimalTriangle[] triangles
