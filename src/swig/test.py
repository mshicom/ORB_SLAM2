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


#%%
base_dir = "/home/nubot/data/workspace/ORB_SLAM2/"
sys = orbslam.System( base_dir + "Vocabulary/ORBvoc.txt",
                      base_dir + "Examples/Monocular/KITTI00-02.yaml",
                      orbslam.System.MONOCULAR,
                      True)
