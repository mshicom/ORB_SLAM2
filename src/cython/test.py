#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 17 18:13:30 2016

@author: kaihong
"""
import numpy as np
import scipy
import matplotlib.pyplot as plt

from orb_slam import *

import sys
sys.path.append("/home/kaihong/workspace/gltes")
sys.path.append("/home/nubot/data/workspace/gltes")
from tools import *
from vtk_visualizer import plotxyz

if __name__ == '__main__':
#    frames, wGc, K, _ = loaddata1()

    from orb_kfs import loadImageFromBag
    image_set, K = loadImageFromBag()
    keys= sorted(image_set.keys())
    frames = [image_set[key] for key in keys]

    h,w = frames[0].shape

    K = np.ascontiguousarray(K,'f')
    frames = [np.ascontiguousarray(f, np.uint8) for f in frames]
#%% run sequence
    slam = pySystem("/home/nubot/rosmake_ws/sandbox/ORB_SLAM2/bumblebee.yaml")

    for f, ts in zip(frames, keys):
        print ts
        print slam.TrackMonocular(f,ts)
    kfs = slam.GetAllKeyFrames()
    mps = slam.GetAllMapPoints()

#%% show keyframes

    for f in kfs:
        pis(image_set[f.mTimeStamp])
        plt.pause(0.01)
        plt.waitforbuttonpress()

#%% show 3d map-points
   p3d= np.vstack([mp.getWorldPos().ravel() for mp in mps])
   plotxyz(p3d)
