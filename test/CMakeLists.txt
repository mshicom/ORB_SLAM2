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



add_executable(test_orb test_orb.cpp)
target_link_libraries(test_orb ORB_SLAM2 pthread gmock ) #gtest_main
add_test(test_orb ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/test_orb)

