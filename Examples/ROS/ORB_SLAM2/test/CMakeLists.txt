# gtest part


include_directories(../src)

# A hack of gmock due to its conflict with catkin
set(gmock_SOURCE_DIR ../../../../test/gtest/googlemock)
include_directories(${gmock_SOURCE_DIR}/include
                    ${gmock_SOURCE_DIR}
                    ${gmock_SOURCE_DIR}/../googletest/include
                    ${gmock_SOURCE_DIR}/../googletest)

find_package(catkin REQUIRED COMPONENTS
             cpp_common rostime roscpp_traits
             roscpp_serialization sensor_msgs rosbag tf nav_msgs tf_conversions)
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV 2.4.3 REQUIRED)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${PROJECT_SOURCE_DIR}/../../../build/test/libgmock.a
ceres
glog
)

add_executable(test_calib CamOdoCalibration_test.cc ../src/CamOdoCalibration.cc)
target_link_libraries(test_calib  ${LIBS})

add_executable(test_tf test_tf.cpp ../src/CamOdoCalibration.cc)
target_link_libraries(test_tf  ${LIBS} ${catkin_LIBRARIES} )

