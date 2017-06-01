%module(threads="1") orbslam
%include "stl.i"
%include "cv_mat.i"

%{
#include "System.h"
#include "MapPoint.h"

#include "KeyFrame.h"
#include "Map.h"

%}

%rename("$ignore", regextarget=1) "(.*)Mutex(.*)";// ignore all the Mutex related function/member
%include "MapPoint.h"

namespace ORB_SLAM2
{
    extern class KeyFrame;
}
%include "Map.h"

namespace std {
 %template(MapPointVector) vector<ORB_SLAM2::MapPoint*>;
 %template(KeyFrameVector) vector<ORB_SLAM2::KeyFrame*>;
}

%include "System.h"





