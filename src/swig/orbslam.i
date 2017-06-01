%module orbslam
%include "std_string.i"
%include "std_vector.i"
%{
#include "System.h"
#include "MapPoint.h"
%}

namespace ORB_SLAM2 {

extern class MapPoint;

}

%include "System.h"



