#from eigency.core cimport *
from libcpp.deque cimport deque
from libcpp.vector cimport vector
from libcpp.set cimport set
from libcpp.list cimport list
from libcpp.pair cimport pair
from libcpp.memory cimport shared_ptr
from libcpp cimport bool
from libcpp.string cimport string
from opencv cimport *
#from KeyFrame cimport *

cdef extern from "../../include/Viewer.h" namespace "ORB_SLAM2" nogil:
    cppclass Viewer:
        pass

cdef extern from "../../Thirdparty/DBoW2/DBoW2/BowVector.h" namespace "DBoW2":
    cppclass BowVector:
        pass

cdef extern from "../../Thirdparty/DBoW2/DBoW2/FeatureVector.h" namespace "DBoW2":
    cppclass FeatureVector:
        pass

cdef extern from "../../include/ORBextractor.h" namespace "ORB_SLAM2":
    cppclass ORBextractor:
        void extract "operator()"(Mat image, Mat mask,  vector[KeyPoint]& keypoints, Mat descriptors)
        ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFAST)

cdef extern from "../../include/ORBmatcher.h" namespace "ORB_SLAM2":
    cppclass ORBmatcher:
        ORBmatcher(float nnratio, bool checkOri)
        int DescriptorDistance(Mat &a, Mat &b)
        int SearchByProjection(Frame &F, const vector[MapPoint*] &vpMapPoints, const float th)
        int SearchByProjection(Frame &F, const vector[MapPoint*] &vpMapPoints)
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const set[MapPoint*] &sAlreadyFound, const float th, const int ORBdist)
        int SearchByProjection(KeyFrame* pKF, Mat Scw, const vector[MapPoint*] &vpPoints, vector[MapPoint*] &vpMatched, int th)
        int SearchByBoW(KeyFrame *pKF, Frame &F, vector[MapPoint*] &vpMapPointMatches)
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, vector[MapPoint*] &vpMatches12)
        int SearchForInitialization(Frame &F1, Frame &F2, vector[Point2f] &vbPrevMatched, vector[int] &vnMatches12, int windowSize)
        int SearchForInitialization(Frame &F1, Frame &F2, vector[Point2f] &vbPrevMatched, vector[int] &vnMatches12)
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, Mat F12, vector[pair[size_t, size_t]] &vMatchedPairs, const bool bOnlyStereo)
        int Fuse(KeyFrame* pKF, const vector[MapPoint *] &vpMapPoints, const float th)
        int Fuse(KeyFrame* pKF, const vector[MapPoint *] &vpMapPoints)
        int Fuse(KeyFrame* pKF, Mat Scw, const vector[MapPoint*] &vpPoints, float th, vector[MapPoint *] &vpReplacePoint)

        const int TH_LOW
        const int TH_HIGH
        const int HISTO_LENGTH

cdef extern from "../../include/ORBVocabulary.h" namespace "ORB_SLAM2":
    cppclass ORBVocabulary:
        bool loadFromTextFile(string &filename)

cdef extern from "../../include/Map.h" namespace "ORB_SLAM2" nogil:
    cppclass Map:
        vector[KeyFrame*] GetAllKeyFrames()
        vector[MapPoint*] GetAllMapPoints()
        vector[MapPoint*] GetReferenceMapPoints()

cdef extern from "../../include/MapPoint.h" namespace "ORB_SLAM2" nogil:
    cppclass MapPoint:
        Mat GetWorldPos()
        Mat GetNormal()
        KeyFrame* GetReferenceKeyFrame()
        bool isBad()
        long unsigned int mnId
        long unsigned int nNextId
        long int mnFirstKFid
        long int mnFirstFrame
        int nObs

cdef extern from "../../include/Frame.h" namespace "ORB_SLAM2" nogil:
    cppclass Frame:
        Frame()
        Frame(const Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, Mat &K, Mat &distCoef, const float &bf, const float &thDepth);
        void ExtractORB(int flag, const Mat &im)
        void ComputeBoW()
        void SetPose(Mat Tcw)

        long unsigned int mnId
        vector[MapPoint*] mvpMapPoints
        vector[KeyPoint] mvKeys, mvKeysUn
        Mat mTcw
        double mTimeStamp
        int N


cdef extern from "../../include/KeyFrame.h" namespace "ORB_SLAM2" nogil:
    ctypedef KeyFrame* pKeyFrame
    cppclass KeyFrame:
        KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB)
        bool isBad()
        Mat GetPose()
        Mat GetPoseInverse()
        Mat GetCameraCenter()
        Mat GetStereoCenter()
        Mat GetRotation()
        Mat GetTranslation()
        set[KeyFrame*] GetConnectedKeyFrames()
        vector[KeyFrame*] GetVectorCovisibleKeyFrames()
        vector[KeyFrame*] GetBestCovisibilityKeyFrames(const int &N)
        vector[KeyFrame*] GetCovisiblesByWeight(const int &w)

        set[MapPoint*] GetMapPoints()
        long unsigned int mnId
        const long unsigned int mnFrameId
        const double mTimeStamp

        BowVector mBowVec
        FeatureVector mFeatVec

        void SetPose(const Mat &Tcw)

cdef extern from "../../include/KeyFrameDatabase.h" namespace "ORB_SLAM2" nogil:
    cppclass KeyFrameDatabase:
        KeyFrameDatabase(const ORBVocabulary &voc)
        void add(KeyFrame* pKF)
        void erase(KeyFrame* pKF)
        void clear()
        vector[KeyFrame *] DetectLoopCandidates(KeyFrame* pKF, float minScore)
        vector[KeyFrame*] DetectRelocalizationCandidates(Frame* F)



cdef extern from "../../include/Tracking.h" namespace "ORB_SLAM2" nogil:
    enum eTrackingState "ORB_SLAM2::Tracking::eTrackingState":
        SYSTEM_NOT_READY = -1
        NO_IMAGES_YET = 0
        NOT_INITIALIZED = 1
        OK = 2
        LOST = 3

    cppclass Tracking:
        # variable
        Mat mK, mDistCoef  # Calibration matrix
        float mbf, mThDepth

        eTrackingState mState
        Frame mCurrentFrame, mLastFrame
        ORBextractor *mpORBextractorLeft
        ORBextractor *mpORBextractorRight
        ORBextractor *mpIniORBextractor
        ORBVocabulary *mpORBVocabulary
        KeyFrameDatabase* mpKeyFrameDB
        list[Mat] mlRelativeFramePoses

        # function
        bool Relocalization(Frame &CurrentFrame)
        void InformOnlyTracking(const bool &flag)
        void ChangeCalibration(const string &strSettingPath)
        void Reset()



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
        System(ORBVocabulary* vocabulary, KeyFrameDatabase* database, Map* map,
           const string &strSettingsFile, const eSensor sensor, const bool bUseViewer)
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
        ORBVocabulary* mpVocabulary
        KeyFrameDatabase* mpKeyFrameDatabase

