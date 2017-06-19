#!/usr/bin/env python

import sys;
sys.path.append('/Users/mereweth/snappy/tsdf_catkin_ws/devel/.private/voxblox/lib/')

import voxblox
dir(voxblox.EsdfMap)

try:
    map = voxblox.EsdfMap('THIS_DOES_NOT_EXIST')
except RuntimeError as e:
    print(e)

#map = voxblox.EsdfMap('/Users/mereweth/Desktop/terra_bella_esdf.proto')
map = voxblox.EsdfMap('/Users/mereweth/Desktop/cow_and_lady_esdf.proto')

import numpy as np

query = np.matrix([[0,0,0.1],
                   [0.1,0,0],
                   [0.1,0.1,0],
                   [0,0.1,0]], dtype='double').T

grad = np.matrix(np.zeros(np.shape(query), dtype='double'))

dist = np.matrix(np.zeros((np.shape(query)[1], 1), dtype='double'))

obs = np.matrix(np.zeros((np.shape(query)[1], 1), dtype='int32'))

map.isObserved(query, obs)
map.getDistanceAtPosition(query, dist, obs)
map.getDistanceAndGradientAtPosition(query, dist, grad, obs)

num_pts = 8021
slice_pos = np.matrix(np.zeros((3, num_pts)))
slice_dist = np.matrix(np.zeros((np.shape(slice_pos)[1], 1), dtype='double'))

map.coordPlaneSliceGetDistance(2, # xy plane
                               0.5, # z
                               slice_pos,
                               slice_dist)

try:
  import IPython; IPython.embed()

except:
  import code
  code.interact(local=dict(globals(), **locals()))
