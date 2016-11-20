#include "CamOdoCalibration.h"
#include "EigenUtils.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <string.h>
#include <cmath>
#include <map>

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> vector_Affine3d;
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vector_Matrix4d;

double random(const double& a, const double& b) { return static_cast<double>(rand()) / RAND_MAX * (b - a) + a; }
double d2r(double deg) { return deg / 180.0 * M_PI; }

TEST(trytf, genTrajectory) {
  ros::init(std::map<std::string, std::string>(), "test");
  ros::start();
  ros::NodeHandle nodeHandler;

  tf::TransformBroadcaster tfbroadcaster;
  tf::Transform odm_tf, cam_tf;

  const double radius = 1;
  const int step = 100;
  vector_Affine3d odm_abs_poses, cam_abs_poses;
  vector_Matrix4d odm_rel_poses_m, cam_rel_poses_m;

  // the transform from camera to base_link
  Eigen::Affine3d oTc;
  oTc = Eigen::Translation3d(0, -0.1, 0)                                   //
        * Eigen::AngleAxisd(-M_PI / 180.0 * 15, Eigen::Vector3d::UnitZ())  //
        * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())             //
        * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());             //
  tf::Transform oTc_tf;
  tf::transformEigenToTF(oTc, oTc_tf);
  Eigen::Affine3d oTc_inv = oTc.inverse();

  // make the odom and camera trajectory
  tf::Transform movement_tf;
  Eigen::Affine3d movement;  // o[i]To[i+1]
  odm_tf.setIdentity();
  odm_tf.setOrigin(tf::Vector3(radius, 0, 0));

  for (int i = 0; i < step; i++) {
    movement = Eigen::Translation3d(random(0, 0.05), random(0, 0.05), 0)  //
               * Eigen::AngleAxisd(d2r(random(-10., 10.)), Eigen::Vector3d::UnitZ());
    tf::transformEigenToTF(movement, movement_tf);
    odm_rel_poses_m.emplace_back(movement.matrix());                    // o[i]To[i+1]
    cam_rel_poses_m.emplace_back((oTc_inv * movement * oTc).matrix());  // c[i]Tc[i+1]

    odm_tf = odm_tf * movement_tf;  // wTo oT[o+1]
    odm_abs_poses.emplace_back();
    tf::transformTFToEigen(odm_tf, odm_abs_poses.back());

    cam_abs_poses.emplace_back();
    tf::transformTFToEigen(odm_tf * oTc_tf, cam_abs_poses.back());

    cam_abs_poses.back().translation() *= 0.2;
    cam_rel_poses_m.back().block<3, 1>(0, 3) *= 0.2;
  }

  vector_Matrix4d odm_abs_poses_m, cam_abs_poses_m;
  for (size_t i = 0; i < step; i++) {
    odm_abs_poses_m.emplace_back(odm_abs_poses.at(i).matrix());
    cam_abs_poses_m.emplace_back(cam_abs_poses.at(i).matrix());
  }

  camodocal::CamOdoCalibration calib;
  calib.setVerbose(true);
  // calib.addMotionSegment(odm_abs_poses_m, cam_abs_poses_m, false);
  calib.addMotionSegment(odm_rel_poses_m, cam_rel_poses_m, true);
  Eigen::Matrix4d oTc_est;
  calib.calibrate(oTc_est);
  std::cout << "Est:\n" << oTc_est << std::endl;
  std::cout << "True:\n" << oTc.matrix() << std::endl;

#if 1
  tf::Transform oTc_est_tf;
  tf::transformEigenToTF(Eigen::Affine3d(oTc_est), oTc_est_tf);

  // display in rviz
  ros::Rate r(10);
  for (size_t i = 0; i < step; i++) {
    r.sleep();
    tf::transformEigenToTF(odm_abs_poses.at(i), odm_tf);
    tfbroadcaster.sendTransform(tf::StampedTransform(odm_tf, ros::Time::now(), "odom", "base_link"));
#if 0
    tf::transformEigenToTF(cam_abs_poses.at(i), cam_tf);
    tfbroadcaster.sendTransform(tf::StampedTransform(cam_tf, ros::Time::now(), "odom", "camera"));
#else
    tfbroadcaster.sendTransform(tf::StampedTransform(oTc_est_tf, ros::Time::now(), "base_link", "camera_est"));
    tfbroadcaster.sendTransform(tf::StampedTransform(oTc_tf, ros::Time::now(), "base_link", "camera"));

#endif
    ros::spinOnce();
  }
#endif
}
