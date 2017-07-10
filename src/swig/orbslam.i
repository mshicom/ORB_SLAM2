%module(threads="1") orbslam
%include "stl.i"
%include "std_set.i"
%include "cv_mat.i"

%{
#include "System.h"
#include "MapPoint.h"

#include "Map.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "Optimizer.h"
%}

%rename("$ignore", regextarget=1) "(.*)Mutex(.*)";// ignore all the Mutex related function/member
%include "MapPoint.h"

namespace ORB_SLAM2
{
    extern class KeyFrameDatabase;
    extern class Frame;
}
%include "KeyFrame.h"
%include "Map.h"

namespace std {
 %template(MapPointVector) vector<ORB_SLAM2::MapPoint*>;
 %template(KeyFrameVector) vector<ORB_SLAM2::KeyFrame*>;
 %template(KeyPointVector) vector<cv::KeyPoint>;
 %template(FloatVector)    vector<float>;

 %template(KeyFrameSet)   set<ORB_SLAM2::KeyFrame*>;
 %template(MapPointSet)   set<ORB_SLAM2::MapPoint*>;

 %template(KeyFrameMapPointMap) map<ORB_SLAM2::KeyFrame*, size_t>;

}



%include "System.h"
%include "Optimizer.h"






