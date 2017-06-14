/* code in the generated cpp, for wraping opencv */
%{
#define SWIG_FILE_WITH_INIT

#define __USING_CV2__ 0
#if __USING_CV2__
    #include "cv2.cpp"
#else
    #include "cv3.cpp"
#endif
%}

%include "numpy.i"
%init %{
  import_array();
%}

%typemap(in) cv::Mat& (cv::Mat cvmat) {
  if( !pyopencv_to($input, cvmat) ) SWIG_fail;
  $1 = &cvmat;
}
%typemap(out) cv::Mat  {
  $result = pyopencv_from($1);
}


namespace cv {

template<typename _Tp> class Point_
{
public:
    typedef _Tp value_type;

    // various constructors
    Point_();
    Point_(_Tp _x, _Tp _y);
    Point_(const Point_& pt);
    _Tp x, y; //< the point coordinates
};

%template(Point) Point_<int>;
%template(Point2f) Point_<float>;
%template(Point2d) Point_<double>;
typedef Point_<float>  Point2f;

class KeyPoint
{
public:
    KeyPoint();
    KeyPoint(Point2f _pt, float _size, float _angle=-1, float _response=0, int _octave=0, int _class_id=-1);
    KeyPoint(float x, float y, float _size, float _angle=-1, float _response=0, int _octave=0, int _class_id=-1);

    Point2f pt;
    float size;
    float angle;

    float response;
    int octave;
    int class_id;
};
}
