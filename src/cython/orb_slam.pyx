#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
cimport numpy as np # for np.ndarray
from libcpp.vector cimport vector
from cpython.ref cimport PyObject
from libcpp cimport bool
from libcpp.string cimport string
# Function to be called at initialization

import cython

np.import_array()

#from lsd_slam cimport Frame,DepthMap
from opencv cimport *
#from eigency.core cimport *

""" KeyFrame """
cdef pyKeyFrame warpKeyFrame(KeyFrame *ptr):
    if not ptr.isBad():
        kf = pyKeyFrame()
        kf.thisptr = ptr
        return kf
    else:
        return None

cdef object warpKeyFrames(vector[KeyFrame*] kfs):
    return [warpKeyFrame(kf) for kf in kfs]

cdef class pyKeyFrame(object):
    cdef KeyFrame *thisptr
    @property
    def Id(self):
        return self.thisptr.mnId

    @property
    def FrameId(self):
        return self.thisptr.mnFrameId

    @property
    def mTimeStamp(self):
        return self.thisptr.mTimeStamp
    @property
    def isBad(self):
        return self.thisptr.isBad()

    def GetPose(self):
        return pyopencv_from(self.thisptr.GetPose())
    def GetPoseInverse(self):
        return pyopencv_from(self.thisptr.GetPoseInverse())
    def GetCameraCenter(self):
        return pyopencv_from(self.thisptr.GetCameraCenter())
    def GetRotation(self):
        return pyopencv_from(self.thisptr.GetRotation())
    def GetTranslation(self):
        return pyopencv_from(self.thisptr.GetTranslation())

    def GetVectorCovisibleKeyFrames(self):
        return warpKeyFrames(self.thisptr.GetVectorCovisibleKeyFrames())

    def GetBestCovisibilityKeyFrames(self, N):
        return warpKeyFrames(self.thisptr.GetBestCovisibilityKeyFrames(N))

    def GetCovisiblesByWeight(self, w):
        return warpKeyFrames(self.thisptr.GetCovisiblesByWeight(w))

    def GetMapPoints(self):
        cdef set[MapPoint*] mps = self.thisptr.GetMapPoints()
        return [warpMapPoint(mp) for mp in mps]


""" MapPoint """
cdef class pyMapPoint(object):
    cdef MapPoint *thisptr
    @property
    def Id(self):
        return long(self.thisptr.mnId)

    def getWorldPos(self):
        return pyopencv_from(self.thisptr.GetWorldPos())

    def getNormal(self):
        return pyopencv_from(self.thisptr.GetNormal())

    def isBad(self):
        return self.thisptr.isBad()

    def getReferenceKeyFrame(self):
        return warpKeyFrame(self.thisptr.GetReferenceKeyFrame())

    def __repr__(self):
        return "MapPoint(Id=%s, Pos=%s)" % (self.Id, self.getWorldPos().ravel())


cdef pyMapPoint warpMapPoint(MapPoint *ptr):
    if not ptr.isBad():
        mp = pyMapPoint()
        mp.thisptr = ptr
        return mp
    else:
        return None

cdef object warpMapPoints(vector[MapPoint*] mps):
    return [warpMapPoint(mp) for mp in mps]

""" System """
cdef class pySystem(object):
    cdef System *thisptr
    def __init__(self, string strSettingsFile,
                 string strVocFile="/home/nubot/rosmake_ws/sandbox/ORB_SLAM2/Vocabulary/ORBvoc.txt",
                 eSensor sensor = MONOCULAR,
                 bool bUseViewer = 0):
        self.thisptr = new System(strVocFile, strSettingsFile, sensor, bUseViewer)

    def __dealloc__(self):
        self.thisptr.Shutdown()
        del self.thisptr

    def TrackMonocular(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] im, double timestamp):
        cdef Mat image,res;
        pyopencv_to(im, image)
        cTw = self.thisptr.TrackMonocular(image, timestamp)
        return pyopencv_from(cTw)

    def SetLocalizationMode(self, bool on_off=True):
        if on_off:
            self.thisptr.ActivateLocalizationMode()
        else:
            self.thisptr.DeactivateLocalizationMode()

    def Reset(self):
        self.thisptr.Reset()

    def GetState(self):
        return self.thisptr.mpTracker.mState

    def GetAllKeyFrames(self):
        return warpKeyFrames(self.thisptr.mpMap.GetAllKeyFrames())

    def GetAllMapPoints(self):
        return warpMapPoints(self.thisptr.mpMap.GetAllMapPoints())

    def GetReferenceMapPoints(self):
        return warpMapPoints(self.thisptr.mpMap.GetReferenceMapPoints())

def GdbBreak():
    breakpoint()

