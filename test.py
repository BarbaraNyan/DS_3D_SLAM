#!/usr/bin/env python

import os
from time import time
import numpy as np
# from numpy import abs, array, eye, max, tile, vstack, zeros, fromfile, ascontiguousarray, concatenate as np
# from numpy.linalg import norm
import point_cloud

def main():

    dirname = "./pcds/"
    dirname_new_pcds = "./data/pcds/"
    dirname_new_octree = "./data/octree/"
    files = sorted( filter( lambda x: os.path.isfile(os.path.join(dirname, x)),
                            os.listdir(dirname) ) )
    #do it first time
    # load point cloud in format with DS
    # for str in files:
    #     # print(str)
    #     bin_str = (dirname + str).encode('utf-8')
    #     X = point_cloud.LoadPcd(bin_str)
    #     bin_new_str = (dirname_new_pcds + str).encode('utf-8')
    #     point_cloud.SavePcd(bin_new_str, X)
    #     # print(X[0])

    initial_file = (dirname_new_pcds + '/' + files[0]).encode('utf-8')
    octree_file_pcd = (dirname_new_octree + 'octree.pcd').encode('utf-8')
    octree_file_bt = (dirname_new_octree + 'octree.bt').encode('utf-8')

    Octree = point_cloud.Create3DMap(initial_file, dirname_new_pcds.encode('utf-8'), 0.5)
    point_cloud.SavePcd(octree_file_pcd, Octree)
    point_cloud.ConverToOctomap(octree_file_pcd, octree_file_bt)


if __name__ == "__main__":
    main()
    exit()
