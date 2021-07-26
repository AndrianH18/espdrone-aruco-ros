#ifndef ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
#define ARUCO_ROS__ARUCO_ROS_UTILS_HPP_

#include <aruco/aruco.h>
#include <aruco/posetracker.h>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/CameraInfo.h>

#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace aruco_ros
{
/**
   * @brief rosCameraInfo2ArucoCamParams gets the camera intrinsics from a CameraInfo message and copies them
   *                                     to aruco_ros own data structure
   * @param cam_info
   * @param useRectifiedParameters if true, the intrinsics are taken from cam_info.P and the distortion parameters
   *                               are set to 0. Otherwise, cam_info.K and cam_info.D are taken.
   * @return
   */
aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo & cam_info,
                                                     bool useRectifiedParameters);

//FIXME: make parameter const as soon as the used function is also const
tf2::Transform arucoMarker2Tf(const aruco::Marker & marker, bool rotate_marker_axis = true);

tf2::Transform arucoMarkerMap2Tf(const aruco::MarkerMapPoseTracker & poseTracker,
                                 bool rotate_board_axis = true);

tf2::Transform markerPose2Tf(const cv::Mat & Rvec, const cv::Mat & Tvec, bool rotate_marker_axis);

// Euclidean distance between two points
inline double dist(const cv::Point2f & p1, const cv::Point2f & p2)
{
  double x1 = p1.x;
  double y1 = p1.y;
  double x2 = p2.x;
  double y2 = p2.y;

  double dx = x1 - x2;
  double dy = y1 - y2;

  return sqrt(dx * dx + dy * dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
double getFiducialArea(const std::vector<cv::Point2f> & pts);

// estimate reprojection error
double getReprojectionError(
  float fiducialSize,
  const aruco::Marker & marker,
  const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs);
}

namespace tf2
{
void convert(const geometry_msgs::Transform & trans, geometry_msgs::Pose & pose);
void convert(const geometry_msgs::Pose & pose, geometry_msgs::Transform & trans);
void convert(const geometry_msgs::TransformStamped & trans, geometry_msgs::PoseStamped & pose);
void convert(const geometry_msgs::PoseStamped & pose, geometry_msgs::TransformStamped & trans);
}

#endif  // ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
