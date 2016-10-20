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
        with nogil:
            res = self.thisptr.TrackMonocular(image, timestamp)
        return pyopencv_from(res)

    def SetLocalizationMode(self, bool on_off=True):
        if on_off:
            self.thisptr.ActivateLocalizationMode()
        else:
            self.thisptr.DeactivateLocalizationMode()

    def Reset(self):
        self.thisptr.Reset()

    def getState(self):
        return self.thisptr.mpTracker.mState
