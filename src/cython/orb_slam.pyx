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

from opencv cimport *
#from eigency.core cimport *

""" KeyPoint """
from collections import namedtuple
pyPoint2f = namedtuple('pyPoint2f', ['x', 'y'])
pyKeyPoint = namedtuple('pyKeyPoint', ['pt', 'size', 'angle', 'response', 'octave', 'class_id'])
cdef object warpKeyPoint(KeyPoint ptr):
    return pyKeyPoint(pyPoint2f(ptr.pt.x, ptr.pt.y),
                                ptr.size,
                                ptr.angle,
                                ptr.response,
                                ptr.octave,
                                ptr.class_id)

cdef object warpKeyPoints(vector[KeyPoint] kps):
    return [warpKeyPoint(kp) for kp in kps]

""" Frame """
cdef class pyFrame(object):
    cdef Frame *thisptr
    cdef public bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr
    @property
    def mnId(self):
        return self.thisptr.mnId
    @property
    def mTimeStamp(self):
        return self.thisptr.mTimeStamp
    @property
    def mTcw(self):
        return pyopencv_from(self.thisptr.mTcw)
    @property
    def N(self):
        return self.thisptr.N
    @property
    def mvpMapPoints(self):
        return pyMapPoint.warpPtrVector(self.thisptr.mvpMapPoints)
    @property
    def mvKeys(self):
        return warpKeyPoints(self.thisptr.mvKeys)
    @property
    def mvKeysUn(self):
        return warpKeyPoints(self.thisptr.mvKeysUn)

    def setPose(self, np.ndarray Tcw):
        cdef Mat Tcw_
        pyopencv_to(Tcw, Tcw_)
        self.thisptr.SetPose(Tcw_)

    @classmethod
    def create(cls, np.ndarray[np.uint8_t, ndim=2, mode="c"] imGray,
                         double timeStamp,
                         pyORBextractor extractor,
                         pyORBVocabulary voc,
                         np.ndarray K,
                         np.ndarray distCoef,
                         float bf,
                         float thDepth):
        cdef Mat imGray_, K_,  distCoef_;
        pyopencv_to(imGray, imGray_)
        pyopencv_to(K, K_)
        pyopencv_to(distCoef, distCoef_)

        f = pyFrame(isSelfOwned=True)
        f.thisptr = new Frame(imGray_, timeStamp, extractor.thisptr, voc.thisptr,
                              K_, distCoef_, bf, thDepth)
        return f

    @staticmethod
    cdef warpPtr(Frame *ptr):
        if ptr!= NULL:
            f = pyFrame()
            f.thisptr = ptr
            return f
        else:
            return None

""" KeyFrame """
cdef class pyKeyFrame(object):
    cdef KeyFrame *thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

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
        return pyKeyFrame.warpPtrVector(self.thisptr.GetVectorCovisibleKeyFrames())

    def GetBestCovisibilityKeyFrames(self, N):
        return pyKeyFrame.warpPtrVector(self.thisptr.GetBestCovisibilityKeyFrames(N))

    def GetCovisiblesByWeight(self, w):
        return pyKeyFrame.warpPtrVector(self.thisptr.GetCovisiblesByWeight(w))

    def GetMapPoints(self):
        cdef set[MapPoint*] mps = self.thisptr.GetMapPoints()
        return [pyMapPoint.warpPtr(mp) for mp in mps]

    @staticmethod
    cdef object warpPtr(KeyFrame *ptr):
        if ptr!= NULL and not ptr.isBad():
            kf = pyKeyFrame()
            kf.thisptr = ptr
            return kf
        else:
            return None
    @staticmethod
    cdef object warpPtrVector(vector[KeyFrame*] kfs):
        return [pyKeyFrame.warpPtr(kf) for kf in kfs]


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
        return pyKeyFrame.warpPtr(self.thisptr.GetReferenceKeyFrame())

    def __repr__(self):
        return "MapPoint(Id=%s, Pos=%s)" % (self.Id, self.getWorldPos().ravel())

    @staticmethod
    cdef warpPtr(MapPoint *ptr):
        if ptr!=NULL and not ptr.isBad():
            mp = pyMapPoint()
            mp.thisptr = ptr
            return mp
        else:
            return None

    @staticmethod
    cdef warpPtrVector(vector[MapPoint*] mps):
        return [pyMapPoint.warpPtr(mp) for mp in mps]


""" ORBextractor """
cdef class pyORBextractor:
    cdef ORBextractor *thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

    def extract(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] image,
                np.ndarray[np.uint8_t, ndim=2, mode="c"] mask = None):
        cdef Mat cv_image, cv_mask;
        cdef Mat descriptors;
        cdef vector[KeyPoint] keypoints,

        pyopencv_to(image, cv_image)
        if mask is not None:
            pyopencv_to(mask, cv_mask)
        self.thisptr.extract(cv_image, cv_mask, keypoints, descriptors)

        keypoints_list = [warpKeyPoint(p) for p in keypoints]
        return keypoints_list, pyopencv_from(descriptors)

    @classmethod
    def create(cls, int nfeatures=2000, float scaleFactor=1.2,
                 int nlevels = 8, int iniThFAST=20, int minThFAST=7):
        extractor = pyORBextractor(True)
        extractor.thisptr = new ORBextractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST)
        return extractor

    @staticmethod
    cdef warpPtr(ORBextractor *ptr):
        extractor = pyORBextractor(False)
        extractor.thisptr = ptr
        return extractor

""" ORBmatcher """
cdef class pyORBmatcher:
    cdef ORBmatcher *thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

    def SearchByProjection(self, pyFrame CurrentFrame, pyFrame LastFrame):
        nmatches = self.thisptr.SearchByProjection(CurrentFrame.thisptr[0],
                                                   LastFrame.thisptr[0],
                                                   0.3, True)
        return nmatches

    @classmethod
    def create(cls, float nnratio=0.6, bool checkOri=True):
        matcher = pyORBmatcher(True)
        matcher.thisptr = new ORBmatcher(nnratio, checkOri)
        return matcher

    @staticmethod
    cdef warpPtr(ORBmatcher *ptr):
        matcher = pyORBmatcher(False)
        matcher.thisptr = ptr
        return matcher


""" ORBVocabulary """
cdef class pyORBVocabulary:
    cdef ORBVocabulary* thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

    @classmethod
    def create(cls, string filename):
        print "Loading ORB Vocabulary. This could take a while..."
        vocabulary = pyORBVocabulary(True)
        vocabulary.thisptr = new ORBVocabulary()
        vocabulary.thisptr.loadFromTextFile(filename)
        return vocabulary

    @staticmethod
    cdef warpPtr(ORBVocabulary *ptr):
        vocabulary = pyORBVocabulary(False)
        vocabulary.thisptr = ptr
        return vocabulary

""" KeyFrameDatabase """
cdef class pyKeyFrameDatabase:
    cdef KeyFrameDatabase* thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr
    def add(self, pyKeyFrame pKF):
        self.thisptr.add(pKF.thisptr)

    def erase(self, pyKeyFrame pKF):
        self.thisptr.erase(pKF.thisptr)

    def clear(self):
        self.thisptr.clear()

    def DetectLoopCandidates(self, pyKeyFrame pKF, float minScore):
        return pyKeyFrame.warpPtrVector(self.thisptr.DetectLoopCandidates(pKF.thisptr, minScore))

    def DetectRelocalizationCandidates(self, pyFrame pF):
        return pyKeyFrame.warpPtrVector(self.thisptr.DetectRelocalizationCandidates(pF.thisptr))

    @classmethod
    def create(cls, pyORBVocabulary vocabulary):
        database = pyKeyFrameDatabase(True)
        database.thisptr = new KeyFrameDatabase(vocabulary.thisptr[0])
        return database

    @staticmethod
    cdef warpPtr(KeyFrameDatabase *ptr):
        database = pyKeyFrameDatabase(False)
        database.thisptr = ptr
        return database

""" Tracking """
from enum import Enum
class TrackingState(Enum):
    SYSTEM_NOT_READY = -1
    NO_IMAGES_YET = 0
    NOT_INITIALIZED = 1
    OK = 2
    LOST = 3

cdef class pyTracking:
    cdef Tracking* thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

    @property
    def mState(self):
        return TrackingState(self.thisptr.mState)
    @property
    def mCurrentFrame(self):
        return pyFrame.warpPtr(&self.thisptr.mCurrentFrame)
    @property
    def mLastFrame(self):
        return pyFrame.warpPtr(&self.thisptr.mLastFrame)
    @property
    def mlRelativeFramePoses(self):
        return [pyopencv_from(p) for p in self.thisptr.mlRelativeFramePoses]


    def makeFrame(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] imGray, double timestamp):
        cdef Mat imGray_
        pyopencv_to(imGray, imGray_)

        cdef Frame* pf = new Frame(imGray_, timestamp,
                                   self.thisptr.mpORBextractorLeft,
                                   self.thisptr.mpORBVocabulary,
                                   self.thisptr.mK,   self.thisptr.mDistCoef,
                                   self.thisptr.mbf,  self.thisptr.mThDepth)
        f = pyFrame.warpPtr(pf)
        f.isSelfOwned = True
        return f

    def Relocalization(self, pyFrame frame):
        cdef bool isSucess = self.thisptr.Relocalization(frame.thisptr[0])
        return isSucess

    def Reset(self):
        self.thisptr.Reset()

    @staticmethod
    cdef warpPtr(Tracking *ptr):
        tracker = pyTracking(False)
        tracker.thisptr = ptr
        return tracker


""" Map """
cdef class pyMap:
    cdef Map* thisptr
    cdef bool isSelfOwned
    def __init__(self, isSelfOwned = False):
        self.isSelfOwned = isSelfOwned
    def __dealloc__(self):
        if self.isSelfOwned:
            del self.thisptr

    def GetAllKeyFrames(self):
        return pyKeyFrame.warpPtrVector(self.thisptr.GetAllKeyFrames())

    def GetAllMapPoints(self):
        return pyMapPoint.warpPtrVector(self.thisptr.GetAllMapPoints())

    def GetReferenceMapPoints(self):
        return pyMapPoint.warpPtrVector(self.thisptr.GetReferenceMapPoints())

    @classmethod
    def create(cls):
        map_ = pyMap(True)
        map_.thisptr = new Map()
        return map_

    @staticmethod
    cdef warpPtr(Map *ptr):
        map_ = pyMap(False)
        map_.thisptr = ptr
        return map_

""" System """
cdef class pySystem(object):
    cdef System *thisptr
    cdef public pyORBVocabulary vocabulary
    cdef public pyKeyFrameDatabase database
    cdef public pyMap map_
    cdef public pyTracking tracker
    def __init__(self, pyORBVocabulary vocabulary,
                       pyKeyFrameDatabase database,
                       pyMap map_,
                       string strSettingsFile,
                       eSensor sensor = MONOCULAR):
        if not os.path.isfile(strSettingsFile):
            raise RuntimeError("path not correct")

        self.thisptr = new System(vocabulary.thisptr, database.thisptr, map_.thisptr,
                                  strSettingsFile, sensor, 0)
        self.vocabulary = vocabulary
        self.database = database
        self.map_ = map_
        self.tracker = pyTracking.warpPtr(self.thisptr.mpTracker)

    def __dealloc__(self):
        if self.thisptr != NULL:
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
        return self.map_.GetAllKeyFrames()

    def GetAllMapPoints(self):
        return self.map_.GetAllMapPoints()

    def GetReferenceMapPoints(self):
        return self.map_.GetReferenceMapPoints()


def GdbBreak():
    breakpoint()

