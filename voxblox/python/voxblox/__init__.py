# TODO(mereweth@jpl.nasa.gov) - this is a bit convoluted but it works
# we are importing all from the shared libraries that we built using pybind11
try:
    from voxblox_tango_interfacepy import *
except ImportError:
    pass

# import from voxbloxpy second to redefine any classes/functions duplicated in voxblox_tango_interfacepy
from voxbloxpy import *
