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

if __name__ == '__main__':
#    frames, wGc, K, _ = loaddata1()

    from orb_kfs import loadImageFromBag
    image_set, K = loadImageFromBag()
    keys= sorted(image_set.keys())
    frames = [image_set[key] for key in keys]

    h,w = frames[0].shape

    K = np.ascontiguousarray(K,'f')
    frames = [np.ascontiguousarray(f, np.uint8) for f in frames]
#    wGc = [np.ascontiguousarray(relPos(wGc[0], g), np.double) for g in wGc]
#%%

    slam = pySystem("/home/nubot/rosmake_ws/sandbox/ORB_SLAM2/bumblebee.yaml")
#    slam.setGradThreshold(10)

    for f, ts in zip(frames, keys):
        print slam.TrackMonocular(f,ts)
        plt.pause(0.5)
    plt.waitforbuttonpress()

