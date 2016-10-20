#from eigency.core cimport *
from libcpp.deque cimport deque
from libcpp.vector cimport vector
from libcpp.memory cimport shared_ptr
from libcpp cimport bool
from libcpp.string cimport string
from opencv cimport *

cdef extern from "../../include/Viewer.h" namespace "ORB_SLAM2" nogil:
    cppclass Viewer:
        pass

cdef extern from "../../include/Map.h" namespace "ORB_SLAM2" nogil:
    cppclass Map:
        pass

cdef extern from "../../include/Tracking.h" namespace "ORB_SLAM2" nogil:
    enum eTrackingState "ORB_SLAM2::Tracking::eTrackingState":
        SYSTEM_NOT_READY = -1
        NO_IMAGES_YET = 0
        NOT_INITIALIZED = 1
        OK = 2
        LOST = 3

    cppclass Tracking:
        eTrackingState mState
        pass

cdef extern from "../../include/LocalMapping.h" namespace "ORB_SLAM2" nogil:
    cppclass LocalMapping:
        pass


cdef extern from "../../include/LoopClosing.h" namespace "ORB_SLAM2" nogil:
    cppclass LoopClosing:
        pass

cdef extern from "../../include/System.h" namespace "ORB_SLAM2" nogil:

    enum eSensor "ORB_SLAM2::System::eSensor":
        MONOCULAR "ORB_SLAM2::System::eSensor::MONOCULAR" = 0
        STEREO "ORB_SLAM2::System::eSensor::STEREO" = 1
        RGBD "ORB_SLAM2::System::eSensor::RGBD" = 2

    cppclass System:
        System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer)
        Mat TrackStereo(const Mat &imLeft, const Mat &imRight, const double &timestamp)
        Mat TrackRGBD(const Mat &im, const Mat &depthmap, const double &timestamp);
        Mat TrackMonocular(const Mat &im, const double &timestamp);
        void ActivateLocalizationMode()
        void DeactivateLocalizationMode()
        void Reset()
        void Shutdown()

        Map* mpMap
        Tracking* mpTracker
        LocalMapping* mpLocalMapper
        LoopClosing* mpLoopCloser


