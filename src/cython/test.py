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
from vtk_visualizer import plotxyz, get_vtk_control
viz = get_vtk_control()

def CreateAxes(pose, length):
    "Create a coordinate axes system with a given length of the axes"
    axesActor = vtk.vtkAxesActor()
    axesActor.AxisLabelsOff()
    axesActor.SetTotalLength(length, length, length)
    m = vtk.vtkMatrix4x4()
    m.DeepCopy(pose.ravel().tolist())
    axesActor.SetUserMatrix(m)
    return axesActor

def plotPoses(poses, length=1):
    for Twc in poses:
        if not Twc is None:
            viz.AddActor(CreateAxes(Twc, length))


def testORBextractor(im):
    ext = pyORBextractor.create(2000)
    kps, desc = ext.extract(im)
    plt.imshow(im)
    x = [ p.pt.x for p in kps]
    y = [ p.pt.y for p in kps]
    plt.plot(x,y,'b.')
    print "total points extracted:", len(kps)
#testORBextractor(frames[0])
extractor = pyORBextractor.create(2000)

def testORBVocabulary():
    if os.path.isfile("/home/nubot/data/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt"):
        filename = "/home/nubot/data/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt"
    elif os.path.isfile("/home/kaihong/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt"):
        filename = "/home/kaihong/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt"
    else:
        raise RuntimeError("path not correct")
    return pyORBVocabulary.create(filename)
vocabulary = testORBVocabulary()

def testFrame(im1, K):
    distCoef = np.array([0,0,0,0],'d')
    bf,thDepth = 0.0, 1.0
    f1 = pyFrame.create(im1,  0.1,
                         extractor, vocabulary,
                         K,  distCoef,
                         bf, thDepth)
    plt.imshow(im1)
    kps = np.array([(p.pt.x, p.pt.y) for p in f1.mvKeys]).T
    plt.plot(kps[0], kps[1], 'b.')
#testFrame(frames[0], K)

def testORBmatcher(im1, Tcw1, im2, Tcw2, K):
    #im1, Tcw1, im2, Tcw2, K = frames[0], inv(wGc[0]), frames[1], inv(wGc[1]), K
    distCoef = np.array([0,0,0,0],'d')
    bf,thDepth = 0.0, 1.0
    flast = pyFrame.create(im1,  0.1,
                         extractor, vocabulary,
                         K,  distCoef,
                         bf, thDepth)
    flast.setPose(Tcw1)

    fnew = pyFrame.create(im2,  0.2,
                         extractor, vocabulary,
                         K,  distCoef,
                         bf, thDepth)
    fnew.setPose(Tcw2)

    matcher = pyORBmatcher.create()
    matches = matcher.SearchByProjection(fnew, flast)
    print "matched pairs:", matches
#testORBmatcher(frames[0], inv(wGc[0]), frames[1], inv(wGc[1]), K)

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
    slam = pySystem("/home/nubot/data/workspace/ORB_SLAM2/bumblebee.yaml",
                    "/home/nubot/data/workspace/ORB_SLAM2/Vocabulary/ORBvoc.txt")

    track_rec = []
    for f, ts in zip(frames, range(len(frames))):
        res = slam.TrackMonocular(f,ts)
        track_rec.append((ts, res))
        print res

    kfs = slam.GetAllKeyFrames()
    mps = slam.GetAllMapPoints()

#%% show keyframes
    for f in kfs:
        pis(image_set[f.mTimeStamp])
        plt.pause(0.01)
        plt.waitforbuttonpress()

#%% show 3d map-points
    p3d = np.vstack([mp.getWorldPos().ravel() for mp in mps if not mp.isBad()])
    plotxyz(p3d)

    ts,poses = zip(*track_rec)
    plotPoses(poses,0.05)

#%% localization mode
    slam.SetLocalizationMode(1)
    slam.TrackMonocular(f,ts)

#%%

