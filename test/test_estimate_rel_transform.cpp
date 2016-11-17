#include "System.h"
#include "gmock/gmock.h"  // already contain gtest

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>



TEST(test, foo) {
    // read in the pos
    ifstream myfile("/home/kaihong/workspace/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/KeyFrameTrajectory.txt");
    ASSERT_TRUE(myfile.is_open());

    std::vector<g2o::SE3Quat> cam, odm;
    double timestamp;
    g2o::Vector7d p;
    g2o::Vector7d q;
    while (!myfile.eof()) {
        myfile >> timestamp >> p(0) >> p(1) >> p(2) >> p(3) >> p(4) >> p(5) >> p(6)
                            >> q(0) >> q(1) >> q(2) >> q(3) >> q(4) >> q(5) >> q(6);

        cam.push_back(g2o::SE3Quat(p));
        odm.push_back(g2o::SE3Quat(q));
    }
    myfile.close();
}

TEST(test, foo2)
{
    // read in the pos
    ifstream myfile("/home/kaihong/workspace/ORB_SLAM2/Examples/ROS/ORB_SLAM2/build/KeyFrameTrajectory.txt");
    ASSERT_TRUE(myfile.is_open());

    std::vector<Eigen::Vector3d> cam, odm;
    double timestamp;
    Eigen::Vector3d p;
    Eigen::Vector3d q;
    double foo;
    while (!myfile.eof()) {
        myfile >> timestamp >> p(0) >> p(1) >> p(2) >> foo >> foo >> foo >> foo
                            >> q(0) >> q(1) >> q(2) >> foo >> foo >> foo >> foo;

        cam.push_back(p);
        odm.push_back(q);
    }
    myfile.close();
    int N = cam.size();
    Eigen::MatrixXd x(3,N),y(3,N);
    for(int i=0;i<N;i++){
        y.col(i) = cam[i];
        x.col(i) = odm[i];
    }

    std::cout<<Eigen::umeyama(x,y,true);    // cRx+t=y

}
