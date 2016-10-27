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
import os
np.import_array()

#from lsd_slam cimport Frame,DepthMap
from opencv cimport *
#from eigency.core cimport *

""" KeyFrame """
cdef pyKeyFrame warpKeyFrame(KeyFrame *ptr):
    if ptr!= NULL and not ptr.isBad():
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
    if ptr!=NULL and not ptr.isBad():
        mp = pyMapPoint()
        mp.thisptr = ptr
        return mp
    else:
        return None

cdef object warpMapPoints(vector[MapPoint*] mps):
    return [warpMapPoint(mp) for mp in mps]

""" System """
cdef class pySystem(object):
    cdef System *thisptr[2]
    def __init__(self, string strSettingsFile,
                 string strVocFile,
                 eSensor sensor = MONOCULAR):
        if not (os.path.isfile(strSettingsFile) and os.path.isfile(strVocFile)):
            raise RuntimeError("path not correct")
        self.thisptr[0] = new System(strVocFile, strSettingsFile, sensor, 0)
        self.thisptr[1] = new System(strVocFile, strSettingsFile, sensor, 0,
                            self.thisptr[0].mpKeyFrameDatabase,
                            self.thisptr[0].mpMap)
        self.thisptr[1].mpTracker.InformWarmStarted()

    def __dealloc__(self):
        self.thisptr[0].Shutdown()
        self.thisptr[1].Shutdown()
#        del self.thisptr[0]
#        del self.thisptr[1]
#        del self.mapptr
#        del self.dbptr

    def TrackMonocular(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] im, double timestamp, int which=0):
        cdef Mat image,res;
        pyopencv_to(im, image)
        cTw = self.thisptr[which].TrackMonocular(image, timestamp)
        return pyopencv_from(cTw)

    def SetLocalizationMode(self, bool on_off=True, int which=0):
        if on_off:
            self.thisptr[which].ActivateLocalizationMode()
        else:
            self.thisptr[which].DeactivateLocalizationMode()

    def Reset(self, int which=0):
        self.thisptr[which].Reset()

    def GetState(self, int which=0):
        return self.thisptr[which].mpTracker.mState

    def GetAllKeyFrames(self, int which=0):
        return warpKeyFrames(self.thisptr[which].mpMap.GetAllKeyFrames())

    def GetAllMapPoints(self, int which=0):
        return warpMapPoints(self.thisptr[which].mpMap.GetAllMapPoints())

    def GetReferenceMapPoints(self, int which=0):
        return warpMapPoints(self.thisptr[which].mpMap.GetReferenceMapPoints())

    def reloc(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] im, double timestamp, int which=0):
        cdef Mat cv_im
        pyopencv_to(im, cv_im)

        cdef Frame CurrentFrame = self.thisptr[which].mpTracker.makeFrame(cv_im, timestamp)
        cdef bool isSucess = self.thisptr[which].mpTracker.Relocalization(CurrentFrame)
        if isSucess:
            return pyopencv_from(CurrentFrame.mTcw)

def GdbBreak():
    breakpoint()

