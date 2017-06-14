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



