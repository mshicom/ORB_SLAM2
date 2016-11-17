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
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <g2o/types/slam3d/se3quat.h>
#include "../../../include/System.h"
using namespace std;
using namespace Eigen;

class ImageGrabber {
 public:
  ImageGrabber(ORB_SLAM2::System* pSLAM) : mpSLAM(pSLAM) {}

  void GrabImage(const sensor_msgs::ImageConstPtr& msg);

  ORB_SLAM2::System* mpSLAM;
};

class OdomGrabber {
 public:
  void receiveOdom(const nav_msgs::OdometryConstPtr& msg) {
    tf::StampedTransform t;
    t.stamp_ = msg->header.stamp;
    t.frame_id_ = msg->header.frame_id;
    t.child_frame_id_ = "base_link";
    tf::poseMsgToTF(msg->pose.pose, t);
    cache.insertData(tf::TransformStorage(t, 1, 0));
    std::cout << "At " << t.stamp_.toSec() <<" [now:"<<ros::Time::now().toSec()<<"] insert tf:\n"
              << Eigen::Map<Eigen::Vector3d>(t.getOrigin()) << std::endl;
  }

  bool queryPos(double timestamp, g2o::SE3Quat& quat) {
    tf::TransformStorage out;
    std::string err;
    bool is_success = cache.getData(ros::Time(timestamp), out, &err);
    if (is_success) {
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      tf::quaternionTFToEigen(out.rotation_, q);
      quat.setRotation(q);
      tf::vectorTFToEigen(out.translation_, t);
      quat.setTranslation(t);
      std::cout << "At " << timestamp <<" query tf:\n" << t << std::endl;
    } else
      std::cout << err << std::endl;
    return is_success;
  }

 protected:
  tf::TimeCache cache;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Mono");
  ros::start();

  if (argc != 3) {
    cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
    ros::shutdown();
    return 1;
  }

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

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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
  mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

}
