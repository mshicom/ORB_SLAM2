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
pic_l_path = "/home/nubot/data/Kitti/image_2/%06d.png"
pic_r_path = "/home/nubot/data/Kitti/image_3/%06d.png"

if 1:
  if 0: # MONOCULAR
    tracker = orbslam.System( base_dir + "Vocabulary/ORBvoc.bin",
                              base_dir + "Examples/Monocular/KITTI00-02.yaml",
                              orbslam.System.MONOCULAR,
                              True)
    t = []
    for i in range(50):
      im = cv2.imread( pic_l_path % i )
      t.append( tracker.TrackMonocular(im, i) )
      print i
    print t

  else: # STEREO
    tracker = orbslam.System( base_dir + "Vocabulary/ORBvoc.bin",
                              base_dir + "Examples/Stereo/KITTI00-02.yaml",
                              orbslam.System.STEREO,
                              True)
    t = []
    for i in range(50):
      im_l = cv2.imread( pic_l_path % i )
      im_r = cv2.imread( pic_r_path % i )
      t.append( tracker.TrackStereo(im_l, im_r, i) )
      print i
    print t

  mps = [mp for mp in tracker.GetTrackedMapPoints() if mp is not None]
  print "%d map points tracked" % len(mps)
  mp_pos = np.hstack([mp.GetWorldPos() for mp in mps])

  kfs = list(tracker.mpMap.GetAllKeyFrames())
  print "%d key frame in use" % len(kfs)
  kf_pos = [kf.GetPose() for kf in kfs]
