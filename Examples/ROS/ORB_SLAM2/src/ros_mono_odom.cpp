/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <g2o/types/slam3d/se3quat.h>
#include "../../../include/System.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include "CamOdoCalibration.h"
#include "EigenUtils.h"
using namespace std;
using namespace Eigen;

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM), scale(3.81665) {
    Eigen::Matrix4d h;
    h << -0.844318, 0.000418584, 0.535842, 0.232158,   //
        -0.535842, 0.000653704, -0.844318, -0.680315,  //
        -0.0007037, -1, -0.000327639, 0,               //
        0, 0, 0, 1;

    double roll, pitch, yaw;
    roll = atan2(h(2, 1), h(2, 2)) / M_PI * 180;
    pitch = atan2(-h(2, 0), sqrt(h(2, 1) * h(2, 1) + h(2, 2) * h(2, 2))) / M_PI * 180;
    yaw = atan2(h(1, 0), h(0, 0)) / M_PI * 180;
    std::cout << "roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    tf::transformEigenToTF(Eigen::Affine3d(h), mTodm_cam);
  }

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  ORB_SLAM2::System* mpSLAM;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> cam_trajectory;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> odm_trajectory;
  tf::TransformBroadcaster mTfBr;
  tf::Transform mTodm_cam;
  double scale;
};

class OdomGrabber {
 public:
  OdomGrabber() : cache_(ros::Duration(60 * 10)) {
    Eigen::Matrix4d h;
    h << -0.844318, 0.000418584, 0.535842, 0.232158,   //
        -0.535842, 0.000653704, -0.844318, -0.680315,  //
        -0.0007037, -1, -0.000327639, 0,               //
        0, 0, 0, 1;

    oTc.setRotation(Eigen::Quaterniond(h.block<3, 3>(0, 0)));
    oTc.setTranslation(h.block<3, 1>(0, 3));
    cTo = oTc.inverse();
  }

  void receiveOdom(const nav_msgs::OdometryConstPtr& msg) {
    tf::StampedTransform t;
    t.stamp_ = msg->header.stamp;
    t.frame_id_ = "odom";
    t.child_frame_id_ = "base_link";
    tf::poseMsgToTF(msg->pose.pose, t);
    cache_.insertData(tf::TransformStorage(t, 1, 0));
    //    std::cout << "At " << t.stamp_.toSec() << " [now:" << ros::Time::now().toSec() << "] insert tf:\n"
    //              << Eigen::Map<Eigen::Vector3d>(t.getOrigin()) << std::endl;

    mTfBr.sendTransform(t);
  }

  bool queryPos(double timestamp, g2o::SE3Quat& quat) {
    tf::TransformStorage out;
    std::string err;
    bool is_success = cache_.getData(ros::Time(timestamp), out, &err);
    if (is_success) {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      tf::quaternionTFToEigen(out.rotation_, q);
      quat.setRotation(q);
      tf::vectorTFToEigen(out.translation_, t);
      quat.setTranslation(t);
      // std::cout << "At " << timestamp << " query tf:\n" << t << std::endl;
    } else
      std::cout << err << std::endl;
    return is_success;
  }

 protected:
  tf::TimeCache cache_;
  tf::TransformBroadcaster mTfBr;
  g2o::SE3Quat oTc, cTo;
};

class MocapGrabber {
 public:
  MocapGrabber() : cache_(ros::Duration(60 * 10)) {
    Eigen::Matrix4d h;
    h << -0.844318, 0.000418584, 0.535842, 0.232158,   //
        -0.535842, 0.000653704, -0.844318, -0.680315,  //
        -0.0007037, -1, -0.000327639, 0,               //
        0, 0, 0, 1;

    oTc.setRotation(Eigen::Quaterniond(h.block<3, 3>(0, 0)));
    oTc.setTranslation(h.block<3, 1>(0, 3));
    cTo = oTc.inverse();
  }

  void receiveOdom(const geometry_msgs::PoseStampedConstPtr& msg) {
    tf::StampedTransform t;
    t.stamp_ = msg->header.stamp;
    t.frame_id_ = "odom";
    t.child_frame_id_ = "base_link";
    tf::poseMsgToTF(msg->pose, t);
    cache_.insertData(tf::TransformStorage(t, 1, 0));
    //    std::cout << "At " << t.stamp_.toSec() << " [now:" << ros::Time::now().toSec() << "] insert tf:\n"
    //              << Eigen::Map<Eigen::Vector3d>(t.getOrigin()) << std::endl;

    mTfBr.sendTransform(t);
  }

  bool queryPos(double timestamp, g2o::SE3Quat& quat) {
    tf::TransformStorage out;
    std::string err;
    bool is_success = cache_.getData(ros::Time(timestamp), out, &err);
    if (is_success) {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      tf::quaternionTFToEigen(out.rotation_, q);
      quat.setRotation(q);
      tf::vectorTFToEigen(out.translation_, t);
      quat.setTranslation(t);
      // std::cout << "At " << timestamp << " query tf:\n" << t << std::endl;
    } else
      std::cout << err << std::endl;
    return is_success;
  }

 protected:
  tf::TimeCache cache_;
  tf::TransformBroadcaster mTfBr;
  g2o::SE3Quat oTc, cTo;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Mono");
  ros::start();

  if (argc != 3) {
    cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
    ros::shutdown();
    return 1;
  }

  camodocal::CamOdoCalibration calib;

  OdomGrabber ogb;
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
  SLAM.SetOdometryFunctor(&OdomGrabber::queryPos, &ogb);

  ImageGrabber igb(&SLAM);
  ros::NodeHandle nodeHandler;
  message_filters::Subscriber<sensor_msgs::Image> sub(nodeHandler, "/camera/image_raw", 1);
  message_filters::TimeSequencer<sensor_msgs::Image> seq(sub, ros::Duration(0.1), ros::Duration(0.01), 50);
  seq.registerCallback(&ImageGrabber::GrabImage, &igb);

  ros::Subscriber sub_odom = nodeHandler.subscribe("/odometry/filtered", 1, &OdomGrabber::receiveOdom, &ogb);

  ros::spin();

  // Save camera trajectory
  // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  calib.addMotionSegment(igb.odm_trajectory, igb.cam_trajectory, false);
  calib.setVerbose(true);
  Eigen::Matrix4d odmTcam;
  calib.calibrate(odmTcam);
  std::cout << odmTcam << std::endl;

  calib.writeMotionSegmentsToFile("cam_odo_trajectory.txt");

  // Stop all threads
  SLAM.Shutdown();
  ros::shutdown();

  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  double timestamp = cv_ptr->header.stamp.toSec();
  cv::Mat cam_abs_pos = mpSLAM->TrackMonocular(cv_ptr->image, timestamp);  // Tcw

  g2o::SE3Quat odm_abs_pos;
  if (!cam_abs_pos.empty() && mpSLAM->getOdometryInfo(timestamp, odm_abs_pos)) {
    // 1. to Twc
    cam_abs_pos = cam_abs_pos.inv().t();

    // 2. camera
    if (cam_abs_pos.type() == CV_32F) {
      cam_trajectory.push_back(Eigen::Map<Eigen::Matrix4f>(reinterpret_cast<float*>(cam_abs_pos.data)).cast<double>());
    } else if (cam_abs_pos.type() == CV_64F)
      cam_trajectory.push_back(Eigen::Map<Eigen::Matrix4d>(reinterpret_cast<double*>(cam_abs_pos.data)));
    // 3. odometry
    odm_trajectory.push_back(odm_abs_pos.to_homogeneous_matrix());

    tf::Transform tfTwc;
    tf::transformEigenToTF(Eigen::Affine3d(cam_trajectory.back()), tfTwc);
    tfTwc.setOrigin(tfTwc.getOrigin() * scale);
    mTfBr.sendTransform(tf::StampedTransform(tfTwc.inverse(), msg->header.stamp, "ORB_SLAM/Camera", "ORB_SLAM/World"));
    mTfBr.sendTransform(tf::StampedTransform(mTodm_cam, msg->header.stamp, "base_link", "ORB_SLAM/Camera"));
  }
}
