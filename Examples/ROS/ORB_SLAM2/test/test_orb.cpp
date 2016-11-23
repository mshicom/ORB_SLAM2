#include "gmock/gmock.h"  // already contain gtest

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <exception>
#include <iostream>
using namespace testing;

namespace enc = sensor_msgs::image_encodings;
int getCvType(const std::string& encoding) {
    // Check for the most common encodings first
    if (encoding == enc::BGR8) return CV_8UC3;
    if (encoding == enc::MONO8) return CV_8UC1;
    if (encoding == enc::RGB8) return CV_8UC3;
    if (encoding == enc::MONO16) return CV_16UC1;
    if (encoding == enc::BGR16) return CV_16UC3;
    if (encoding == enc::RGB16) return CV_16UC3;
    if (encoding == enc::BGRA8) return CV_8UC4;
    if (encoding == enc::RGBA8) return CV_8UC4;
    if (encoding == enc::BGRA16) return CV_16UC4;
    if (encoding == enc::RGBA16) return CV_16UC4;

    // For bayer, return one-channel
    if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
    if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
    if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
    if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
    if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
    if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
    if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
    if (encoding == enc::BAYER_GRBG16) return CV_16UC1;
    assert(false);
}

class ORBTest : public ::testing::Test {
   protected:
    virtual void SetUp() {
        mbag.open("/media/kaihong/2ADA2A32DA29FAA9/husky_mocap.bag", rosbag::bagmode::Read);

        mview_img = new rosbag::View(mbag, rosbag::TopicQuery("/image_raw"));
        mit_img = mview_img->begin();

        mview_odm = new rosbag::View(mbag, rosbag::TopicQuery("/odometry/filtered"));
        mit_odm = mview_odm->begin();
    }

    virtual void TearDown() {
        delete mview_img;
        delete mview_odm;
        mbag.close();
    }

    bool update_img(cv::OutputArray& img_out) {
        if (mit_img == mview_img->end()) return false;

        sensor_msgs::ImageConstPtr pimg = (mit_img++)->instantiate<sensor_msgs::Image>();
        if (pimg == NULL) return false;

        int type = getCvType(pimg->encoding);
        cv::Mat frame(pimg->height, pimg->width, type, const_cast<uchar*>(&pimg->data[0]), pimg->step);
        frame.copyTo(img_out);
        return true;
    }

    bool update_odom(nav_msgs::Odometry* msg_out) {
        if (mit_odm == mview_odm->end()) return false;

        nav_msgs::OdometryConstPtr pmsg = (mit_odm++)->instantiate<nav_msgs::Odometry>();
        if (pmsg == NULL) return false;
        *msg_out = *pmsg;

        return true;
    }

   public:
    rosbag::Bag mbag;
    rosbag::View* mview_img;
    rosbag::View::iterator mit_img;
    rosbag::View* mview_odm;
    rosbag::View::iterator mit_odm;
};

TEST_F(ORBTest, read) {
    nav_msgs::Odometry msg;
    ASSERT_TRUE(update_odom(&msg));
    std::cout << msg.header.stamp << std::endl;

    cv::Mat im;
    ASSERT_TRUE(update_img(im));
    std::cout << im.rows << "," << im.cols << std::endl;
}
