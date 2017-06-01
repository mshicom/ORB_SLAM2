#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  1 08:46:53 2017

@author: nubot
"""
import numpy as np
import scipy
import matplotlib.pyplot as plt

import orbslam
import cv2
#%%
base_dir = "/home/nubot/data/workspace/ORB_SLAM2/"
pic_path = "/home/nubot/data/Kitti/%06d.png"

if 1:
  tracker = orbslam.System( base_dir + "Vocabulary/ORBvoc.txt",
                        base_dir + "Examples/Monocular/KITTI00-02.yaml",
                        orbslam.System.MONOCULAR,
                        False)
  t = []
  for i in range(50):
    im = cv2.imread(pic_path%i)
    t.append( tracker.TrackMonocular(im, i) )
  print t

  mps = [mp for mp in tracker.GetTrackedMapPoints() if mp is not None]
  print "%d map points tracked" % len(mps)

  kfs = tracker.mpMap.GetAllKeyFrames()
  print "%d key frame in use" % len(kfs)
