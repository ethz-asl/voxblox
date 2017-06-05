#!/usr/bin/env python

import sys;
sys.path.append('/Users/mereweth/snappy/tsdf_catkin_ws/devel/.private/voxblox/lib/')

import voxbloxpy
dir(voxbloxpy.EsdfMap)

map = voxbloxpy.EsdfMap('/Users/mereweth/Desktop/esdf_map.proto')

import numpy as np

query = np.matrix([[0,0,0.1],
                   [0.1,0,0],
                   [0.1,0.1,0],
                   [0,0.1,0]], dtype='double').T

obs = np.matrix(np.zeros(np.shape(query)[1], dtype='int32')).T
map.isObserved(query, obs)
obs

try:
  import IPython; IPython.embed()

except:
  import code
  code.interact(local=dict(globals(), **locals()))
