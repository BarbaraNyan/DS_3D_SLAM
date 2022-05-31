'''A module with utilities for manipulating a point cloud (nx3 numpy array).'''

# IMPORTS ==========================================================================================

# python
import ctypes
from copy import copy
from ctypes import c_char, c_int, c_float, pointer, POINTER, c_wchar
# scipy
from numpy.linalg import norm
from scipy.io import loadmat, savemat
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
from numpy.ctypeslib import ndpointer
from numpy import array, ascontiguousarray, dot, empty, eye, isinf, isnan, logical_and, \
    logical_not, logical_or, ones, repeat, reshape, sum, vstack, zeros

# C BINDINGS =======================================================================================

PointCloudsPython = ctypes.cdll.LoadLibrary(__file__[:-15] + "/build/libPointCloudPython.so")

CopyAndFree = PointCloudsPython.CopyAndFree
CopyAndFree.restype = c_int
CopyAndFree.argtypes = [POINTER(c_float), ndpointer(c_float, flags="C_CONTIGUOUS"), c_int]

PclSavePcd = PointCloudsPython.PclSavePcd
PclSavePcd.restype = c_int
PclSavePcd.argtypes = [POINTER(c_char), ndpointer(c_float, flags="C_CONTIGUOUS"), c_int]

PclLoadPcd = PointCloudsPython.PclLoadPcd
PclLoadPcd.restype = c_int
PclLoadPcd.argtypes = [POINTER(c_char), POINTER(POINTER(c_float)), POINTER(c_int)]

ConvertPcdToOctomap = PointCloudsPython.ConvertPcdToOctomap
ConvertPcdToOctomap.restype = c_int
ConvertPcdToOctomap.argtypes = [POINTER(c_char), POINTER(c_char)]

ICP_main = PointCloudsPython.ICP_main
ICP_main.restype = c_int
ICP_main.argtypes = [POINTER(c_char), POINTER(c_char), c_float, POINTER(POINTER(c_float)), POINTER(c_int)]


# FUNCTIONS ========================================================================================

def SavePcd(fileName, cloud):
    cloud = ascontiguousarray(cloud, dtype='float32')
    print(type(cloud))
    print(type(cloud.shape[0]))
    errorCode = PclSavePcd(fileName, cloud, cloud.shape[0])

    if errorCode < 0:
        raise Exception("Failed to save {}.".format(fileName))


def LoadPcd(fileName):
    nPoints = pointer(c_int(0))
    ppoints = pointer(pointer(c_float(0)))
    errorCode = PclLoadPcd(fileName, ppoints, nPoints)
    # print(errorCode)
    points = ppoints.contents
    nPoints = nPoints.contents.value

    cloud = empty((nPoints, 7), dtype='float32', order='C')
    errorCode = CopyAndFree(points, cloud, nPoints)

    return cloud

def ConvertToOctomap(input, output):
    ConvertPcdToOctomap(input, output)

def Create3DMap(octree_file, dirname, res):
    nPoints = pointer(c_int(0))
    ppoints = pointer(pointer(c_float(0)))
    errorCode = ICP_main(octree_file, dirname, res, ppoints, nPoints)
    points = ppoints.contents
    nPoints = nPoints.contents.value
    OctCloud = empty((nPoints, 7), dtype='float32', order='C')
    errorCode = CopyAndFree(points, OctCloud, nPoints)
    return OctCloud