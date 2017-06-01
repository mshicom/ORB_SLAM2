%module(threads="1") orbslam
%include "stl.i"
%include "cv_mat.i"

%{
#include "System.h"
#include "MapPoint.h"
%}

%rename("$ignore", regextarget=1) "(.*)Mutex(.*)";// ignore all the Mutex related function/member
%include "MapPoint.h"


namespace std {
    %template(MapPointVector) vector<ORB_SLAM2::MapPoint*>;
}
%include "System.h"




