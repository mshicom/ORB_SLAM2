#ifndef CAMODOCALIBRATION_H
#define CAMODOCALIBRATION_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <vector>

namespace camodocal {
/*!
 * \brief The CamOdoCalibration class
 * In the first step, our approach solves for the \b pitch and \b roll
 * components of the camera-odometry rotation by minimizing a
 * least-squares cost function as in [7].
 * In the second step, we solve for the \b yaw component of the
 * camera-odometry rotation, \b the camera-odometry translation and
 * the \b scales for all sparse maps by minimizing another least-squares
 * cost function. As the vehicle motion is planar,the \b z-component
 * of the camera-odometry translation is unobservable and therefore set to zero.
 */
class CamOdoCalibration {
 public:
  CamOdoCalibration();

  bool addMotion(const Eigen::Ref<const Eigen::Matrix4d>& H_odo, const Eigen::Ref<const Eigen::Matrix4d>& H_cam);

  bool addMotionSegment(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& H_odo_,
                        const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& H_cam_,
                        bool is_relative_position = false);

  void getCurrentCameraMotion(Eigen::Vector3d& rotation, Eigen::Vector3d& translation) const;

  bool motionsEnough(void) const;
  size_t getCurrentMotionCount(void) const;
  size_t getMotionCount(void) const;
  void setMotionCount(size_t count);

  bool calibrate(Eigen::Matrix4d& H_cam_odo);

  bool readMotionSegmentsFromFile(const std::string& filename);
  bool writeMotionSegmentsToFile(const std::string& filename) const;

  bool getVerbose(void);
  void setVerbose(bool on = false);

  bool estimate(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs1,
                const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs1,
                const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs2,
                const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs2,
                Eigen::Matrix4d& H_cam_odo, std::vector<double>& scales) const;

 private:
  bool estimateRyx(const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs1,
                   const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs1,
                   const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs2,
                   const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs2,
                   Eigen::Matrix3d& R_yx) const;

  void refineEstimate(
      Eigen::Matrix4d& H_cam_odo, std::vector<double>& scales,
      const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs1,
      const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs1,
      const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& rvecs2,
      const std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>>& tvecs2) const;

  // solve ax^2 + bx + c = 0
  bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const;

  typedef struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d rotation;
    Eigen::Vector3d translation;
  } Motion;

  typedef struct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Motion> odoMotions;
    std::vector<Motion> camMotions;
  } MotionSegment;

  std::vector<MotionSegment> mSegments;

  size_t mMinMotions;

  bool mVerbose;
};
}

#endif
