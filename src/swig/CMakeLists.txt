cmake_minimum_required(VERSION 3.0.1)
project(pyorbslam)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -std=c++11 ")


FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
find_package(OpenCV REQUIRED opencv_core)
FIND_PACKAGE(PythonLibs)


include_directories(
${PYTHON_INCLUDE_PATH}
${OpenCV_INCLUDE_DIRS}
/usr/include/eigen3/
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
)

set(LIBS 
${PYTHON_LIBRARIES}
${OpenCV_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../lib/libORB_SLAM2.so
)

# swig
set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_SOURCE_DIR})
set_source_files_properties(orbslam.i PROPERTIES CPLUSPLUS ON)
set_property(SOURCE orbslam.i PROPERTY SWIG_FLAGS
             ${swig_extra_flags}
             -Wall)
swig_add_module(orbslam python orbslam.i)
set_target_properties(_orbslam PROPERTIES 
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SWIG_OUTDIR})
swig_link_libraries(orbslam ${LIBS})



