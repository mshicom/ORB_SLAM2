#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 23 20:04:45 2016

@author: nubot
"""
import numpy as np
import scipy
import matplotlib.pyplot as plt

from orb_slam import *

import sys
sys.path.append("/home/kaihong/workspace/gltes")
sys.path.append("/home/nubot/data/workspace/gltes")
from tools import *
from test_orb import *
from EpilineCalculator import *
from vtk_visualizer import plotxyz,plotxyzrgb
#%%

from orb_kfs import loadImageFromBag
image_set, K = loadImageFromBag()
keys= sorted(image_set.keys())
frames = [image_set[key] for key in keys]

h,w = frames[0].shape

K = np.ascontiguousarray(K,'f')
frames = [np.ascontiguousarray(f, np.uint8) for f in frames]
    #%%
class KFrame:
    def __init__(self, kf):
        self.wGc = kf.GetPoseInverse()
        self.ts = kf.mTimeStamp
        self.im = image_set[self.ts]
        self.imf = self.im.astype('f')
        dx,dy = np.gradient(self.imf)
        grad = np.sqrt(dx**2 + dy**2)

        h,w = self.im.shape
        u, v = np.meshgrid(range(w),range(h))
        border_width = 5
        mask = reduce(np.logical_and, [w-border_width>u, u>=border_width,
                                       h-border_width>v, v>=border_width, grad>10]) # exclude border pixels
        y,x = np.where(mask)
        self.mask = np.full_like(self.imf, '-1',dtype=np.int)
        self.mask[mask] = np.arange(len(x))

        self.py, self.px = y,x
        self.P = backproject(x, y, K)

    def makePC(self, depth, valid_mask):#
        vm_ind, = np.where(valid_mask)
        p3d = self.P[:, vm_ind]*depth
        I = np.tile(self.imf[self.py[vm_ind], self.px[vm_ind]],(3,1))
        P = np.vstack([p3d, I]).T
        return P      # Nx6

    def growD(self, seed, f0):
        p2d = K.dot(transform(inv(self.wGc), seed))    # seed:=[3,M]
        d = p2d[2].copy()
        p2d = p2d[:2]/d

        # find target points to grow, who are the nearest neighbor to the projected 2d seed points
        tree_ref = scipy.spatial.cKDTree(p2d.T)
        tree_cur = scipy.spatial.cKDTree(np.vstack([self.px, self.py]).T)
        idxInRef = tree_cur.query_ball_tree(tree_ref, 1)
        idxInRef = np.array([ i[0] if i else -1 for i in idxInRef])

        mask = idxInRef!=-1
        target = np.vstack([px[mask], py[mask], d[idxInRef[mask]]])

        ec = EpilineCalculator(target[0], target[1],  inv(f0.wGc).dot(self.wGc),  K)
        v0 = ec.VfromD(target[2])

        sampleRef,sampleCur = ec.createEPLSampler(self.imf, f0.imf)
        vmin, vmax, dmin, dmax, valid_mask = ec.getLimits(self.imf.shape, dmin=0.0, dmax=1e6)

        offset_l = np.arange(-2, 3)
        vres = np.empty_like(v0)
        for pid in range(target.shape[1]):
            ref = sampleRef(offset_l, pid)
            offset_r = v0[pid]+offset_l
            SAD = lambda offset: np.abs(ref - sampleCur(offset+offset_r, pid)).sum()
            sign = -1 if SAD(-1)<SAD(1) else 1

            i = 0   #
            cost = SAD(0)
            while SAD(i+sign) < cost:
                cost = SAD(i+sign)
                i += sign
            vres[pid] = v0[pid]+i
        dres = ec.DfromV(vres)
        return dres, mask

#%%
if __name__ == '__main__':
#    frames, wGc, K, _ = loaddata1()


#%% run sequence
#    base_path = "/home/nubot/rosmake_ws/sandbox/ORB_SLAM2/"
    base_path = "/home/kaihong/workspace/rosbuild_ws/sandbox/ORB_SLAM2/"
    slam = pySystem(base_path+"bumblebee.yaml", base_path+"Vocabulary/ORBvoc.txt")
    for f, ts in zip(frames, keys):
        res = slam.TrackMonocular(f,ts)
        print res
    kfs = slam.GetAllKeyFrames()
    mps = slam.GetAllMapPoints()

#%%
    f0 = kfs[15]
    fnbr = f0.GetBestCovisibilityKeyFrames(10)
    ts = [f0.mTimeStamp] + [f.mTimeStamp for f in fnbr]
    im = [image_set[key] for key in ts]
    wGc = [f0.GetPoseInverse()] + [f.GetPoseInverse() for f in fnbr ]

    mp = f0.GetMapPoints()
    seed= np.vstack([p.getWorldPos().ravel() for p in mp]).T
    plotxyz(seed.T)

    imf0 = im[0].astype('f')

    cur_id = 3
    f1 = fnbr[cur_id-1]
    im1 = image_set[ts[cur_id]]
    imf1 = im1.astype('f')
    dx,dy = np.gradient(imf1)
    grad = np.sqrt(dx**2 + dy**2)


    u, v = np.meshgrid(range(w),range(h))
    border_width = 5
    mask = reduce(np.logical_and, [w-border_width>u, u>=border_width,
                                   h-border_width>v, v>=border_width, grad>10]) # exclude border pixels
    py,px = np.where(mask)
    immask = np.full_like(mask, -1, np.int)
    immask[mask] = np.arange(len(px))


    EpilineDrawer([imf1, imf0], [wGc[cur_id],wGc[0]], K, (359,287), 2)
    a1,a2,a3 = plt.gcf().get_axes()

    a1.plot(px,py,'b.',ms=1)

    p2d = K.dot(transform(inv(wGc[cur_id]), seed))    # seed:=[3,M]
    d = p2d[2].copy()
    p2d = p2d[:2]/d
    a1.plot(p2d[0],p2d[1],'r.' )
    # find target points to grow, who are the nearest neighbor to the projected 2d seed points
    tree_ref = scipy.spatial.cKDTree(p2d.T)
    tree_cur = scipy.spatial.cKDTree(np.vstack([px, py]).T)
    idxInRef = tree_cur.query_ball_tree(tree_ref, 1)
    idxInRef = np.array([ i[0] if i else -1 for i in idxInRef])
    mask = idxInRef!=-1

    target = np.vstack([px[mask], py[mask], d[idxInRef[mask]]])

    ec = EpilineCalculator(target[0], target[1],  inv(wGc[0]).dot(wGc[cur_id]),  K)
    v0 = ec.VfromD(target[2])

    sampleRef,sampleCur = ec.createEPLSampler(imf1, imf0)
    vmin, vmax, dmin, dmax, valid_mask = ec.getLimits(imf1.shape, dmin=0.0, dmax=1e6)

    offset_l = np.arange(-2, 3)
    vres = np.empty_like(v0)
    for pid in range(target.shape[1]):
        ref = sampleRef(offset_l, pid)
        offset_r = v0[pid]+offset_l
        SAD = lambda offset: np.abs(ref - sampleCur(offset+offset_r, pid)).sum()
        sign = -1 if SAD(-1)<SAD(1) else 1

        i,cost = 0,SAD(0)
        while SAD(i+sign) < cost:
            cost = SAD(i+sign)
            i += sign
        vres[pid] = v0[pid]+i

    dres = ec.DfromV(vres)
    tree_tar = scipy.spatial.cKDTree(target[:2].T)
    tree_tar.query_ball_point([359,287],1)
    target[:,38]

    #%%
    kf0 = KFrame(f0)
    kf1 = KFrame(f1)
    seed= np.vstack([p.getWorldPos().ravel() for p in f0.GetMapPoints()]).T
    plotxyzrgb(kf1.makePC(*kf1.growD(seed, kf0)))

    def showMapPoint(kf):
        pf()
        cGw = kf.GetPose()
        mp = np.vstack([p.getWorldPos().ravel() for p in kf.GetMapPoints()]).T
        p2d = K.dot(transform(cGw, mp))
        p2d /= p2d[2]

        plt.imshow(image_set[kf.mTimeStamp])
        plt.plot(p2d[0],p2d[1],'b.')