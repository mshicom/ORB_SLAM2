# gtest part

#add_subdirectory(gtest)
include_directories(../src)

# A hack of gmock due to its conflict with catkin
set(gmock_SOURCE_DIR gtest/googlemock)
include_directories(${gmock_SOURCE_DIR}/include
                    ${gmock_SOURCE_DIR}
                    ${gmock_SOURCE_DIR}/../googletest/include
                    ${gmock_SOURCE_DIR}/../googletest)
add_library(gmock
            ${gmock_SOURCE_DIR}/../googletest/src/gtest-all.cc
            ${gmock_SOURCE_DIR}/src/gmock-all.cc
            ${gmock_SOURCE_DIR}/src/gmock_main.cc)
target_link_libraries(gmock pthread)
# Replace catkin variable
set(GTEST_LIBRARIES gmock)


find_package(catkin REQUIRED COMPONENTS
             cpp_common rostime roscpp_traits
             roscpp_serialization sensor_msgs rosbag tf nav_msgs tf_conversions)
include_directories(${catkin_INCLUDE_DIRS})




#add_executable(test_est test_estimate_rel_transform.cpp)
#target_link_libraries(test_est ORB_SLAM2 pthread gmock) #gtest_main
#add_test(test_est ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_est)
