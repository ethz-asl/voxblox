#!/usr/bin/env python

import numpy as np

import timeit

import sys
sys.path.append(
    '/Users/mereweth/snappy/tsdf_catkin_ws/devel/.private/voxblox/lib/')

import voxblox
dir(voxblox.EsdfMap)

try:
  layer = voxblox.loadEsdfLayer('THIS_DOES_NOT_EXIST')
except RuntimeError as e:
  print(e)

# make sure this throws an exception rather than segfaulting
# try:
  # voxblox_tango_interfacepy.tangoLoadLayer("/Users/mereweth/Desktop/cow_and_lady/cow_and_lady.tsdf.proto")
# except RuntimeError as e:
  # print(e)

# convert Tango TSDF to TSDF and serialize
#ntl = voxblox_tango_interfacepy.tangoLoadLayer("/Users/mereweth/Desktop/map.ntsdf.proto")
# ntl.saveToFile("/Users/mereweth/Desktop/map.tsdf.proto")

# make sure this throws an exception rather than segfaulting
try:
  voxblox.loadTsdfLayer("/Users/mereweth/Desktop/map.ntsdf.proto")
except RuntimeError as e:
  print(e)

tl = voxblox.loadTsdfLayer(
    "/Users/mereweth/Desktop/environments/bldg_344_huge/map_cleared.tsdf.proto")
el = voxblox.EsdfLayer(tl.voxel_size, tl.voxels_per_side)
m = voxblox.EsdfMap(el)
ei = voxblox.EsdfIntegrator(voxblox.EsdfIntegratorConfig(), tl, el)
ei.updateFromTsdfLayerBatch()

b = tl.allocateBlockPtrByCoordinates(np.array([0, 0, 0.5], dtype='double'))
v = b.getVoxelByCoordinates(np.array([0, 0, 0.5], dtype='double'))

v.distance = 0.3
b.set_updated(True)

max_distance_m = 2.0
voxblox.clearSphereAroundPoint(
    np.array([0, 0, 0.5], dtype='double'), 0.5, max_distance_m, el)
voxblox.fillSphereAroundPoint(
    np.array([0, 0, 0.5], dtype='double'), 0.5, max_distance_m, el)

el.saveToFile("/Users/mereweth/Desktop/_test_cow_and_lady.esdf.proto")

layer = voxblox.loadEsdfLayer(
    '/Users/mereweth/Desktop/environments/bldg_344_huge/map_cleared.esdf.proto')
assert(layer is not None)
map = voxblox.EsdfMap(layer)

print map.voxel_size
print map.block_size

import numpy as np

x_ = np.linspace(0.0, 0.2, 100)
y_ = np.linspace(0.0, 0.2, 100)
z_ = np.linspace(0.0, 0.2, 100)
x, y, z = np.meshgrid(x_, y_, z_)
query = np.matrix(np.c_[x.flatten(), y.flatten(), z.flatten()]).T

# query = np.matrix([[0,0,0.1],
#                   [0.1,0,0],
#                   [0.1,0.1,0],
#                   [0,0.1,0]], dtype='double').T

grad = np.matrix(np.zeros(np.shape(query), dtype='double'))

dist = np.matrix(np.zeros((np.shape(query)[1], 1), dtype='double'))

obs = np.matrix(np.zeros((np.shape(query)[1], 1), dtype='int32'))

map.isObserved(query, obs)
map.getDistanceAtPosition(query, dist, obs)


def interp_fun():
  map.getDistanceAndGradientAtPosition(query, dist, grad, obs)


interp_duration = timeit.timeit(interp_fun, number=10) / 10.0
print(interp_duration)

num_pts = 1000 * 1000
slice_pos = np.matrix(np.zeros((3, num_pts)))
slice_dist = np.matrix(np.zeros((np.shape(slice_pos)[1], 1), dtype='double'))


def no_interp_fun():
  map.coordPlaneSliceGetDistance(2,  # xy plane
                                 0.5,  # z
                                 slice_pos,
                                 slice_dist,
                                 num_pts)


no_interp_duration = timeit.timeit(no_interp_fun, number=10) / 10.0
print(no_interp_duration)

try:
  import IPython
  IPython.embed()

except BaseException:
  import code
  code.interact(local=dict(globals(), **locals()))
