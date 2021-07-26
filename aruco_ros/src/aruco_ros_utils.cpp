#include <aruco_ros/aruco_ros_utils.hpp>
#include <ros/console.h>
#include <iostream>

aruco::CameraParameters aruco_ros::rosCameraInfo2ArucoCamParams(
  const sensor_msgs::CameraInfo & cam_info,
  bool useRectifiedParameters)
{
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.height, cam_info.width);

  if (useRectifiedParameters) {
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.P[0];
    cameraMatrix.at<double>(0, 1) = cam_info.P[1];
    cameraMatrix.at<double>(0, 2) = cam_info.P[2];
    cameraMatrix.at<double>(1, 0) = cam_info.P[4];
    cameraMatrix.at<double>(1, 1) = cam_info.P[5];
    cameraMatrix.at<double>(1, 2) = cam_info.P[6];
    cameraMatrix.at<double>(2, 0) = cam_info.P[8];
    cameraMatrix.at<double>(2, 1) = cam_info.P[9];
    cameraMatrix.at<double>(2, 2) = cam_info.P[10];

    for (int i = 0; i < 4; ++i) {
      distorsionCoeff.at<double>(i, 0) = 0;
    }
  } else {
    for (int i = 0; i < 9; ++i) {
      cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.K[i];
    }

    if (cam_info.D.size() == 4) {
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
      }
    } else {
      ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = 0;
      }
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf2::Transform aruco_ros::arucoMarker2Tf(const aruco::Marker & marker, bool rotate_marker_axis)
{
  return aruco_ros::markerPose2Tf(marker.Rvec, marker.Tvec, rotate_marker_axis);
}

tf2::Transform aruco_ros::arucoMarkerMap2Tf(const aruco::MarkerMapPoseTracker & poseTracker,
                                            bool rotate_board_axis)
{
  return aruco_ros::markerPose2Tf(poseTracker.getRvec(), poseTracker.getTvec(), rotate_board_axis);
}

tf2::Transform aruco_ros::markerPose2Tf(const cv::Mat & Rvec,
                                        const cv::Mat & Tvec,
                                        bool rotate_marker_axis)
{
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat Tvec64;
  Tvec.convertTo(Tvec64, CV_64FC1);

  // Rotate axis direction as to fit ROS (?)
  if (rotate_marker_axis) {
    double data[9] = {-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0};
    cv::Mat rotate_to_ros(3, 3, CV_64FC1, data);
    // -1 0 0
    //  0 0 1
    //  0 1 0
    rot = rot * rotate_to_ros.t();
  }

  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                        rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                        rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

  tf2::Vector3 tf_orig(Tvec64.at<double>(0, 0), Tvec64.at<double>(0, 1), Tvec64.at<double>(0, 2));

  tf2::Transform temp = tf2::Transform(tf_rot, tf_orig);
  return temp;
}

void tf2::convert(const geometry_msgs::Transform & trans, geometry_msgs::Pose & pose)
{
  pose.orientation = trans.rotation;
  pose.position.x = trans.translation.x;
  pose.position.y = trans.translation.y;
  pose.position.z = trans.translation.z;
}

void tf2::convert(const geometry_msgs::Pose & pose, geometry_msgs::Transform & trans)
{
  trans.rotation = pose.orientation;
  trans.translation.x = pose.position.x;
  trans.translation.y = pose.position.y;
  trans.translation.z = pose.position.z;
}

void tf2::convert(const geometry_msgs::TransformStamped & trans, geometry_msgs::PoseStamped & pose)
{
  convert(trans.transform, pose.pose);
  pose.header = trans.header;
}

void tf2::convert(const geometry_msgs::PoseStamped & pose, geometry_msgs::TransformStamped & trans)
{
  convert(pose.pose, trans.transform);
  trans.header = pose.header;
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
double aruco_ros::getFiducialArea(const std::vector<cv::Point2f> & pts)
{
  const cv::Point2f & p0 = pts.at(0);
  const cv::Point2f & p1 = pts.at(1);
  const cv::Point2f & p2 = pts.at(2);
  const cv::Point2f & p3 = pts.at(3);

  double a1 = dist(p0, p1);
  double b1 = dist(p0, p3);
  double c1 = dist(p1, p3);

  double a2 = dist(p1, p2);
  double b2 = dist(p2, p3);
  double c2 = c1;

  double s1 = (a1 + b1 + c1) / 2.0;
  double s2 = (a2 + b2 + c2) / 2.0;

  a1 = sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
  a2 = sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
  return a1 + a2;
}

// estimate reprojection error
double aruco_ros::getReprojectionError(
  float fiducialSize,
  const aruco::Marker & marker,
  const cv::Mat & cameraMatrix, const cv::Mat & distCoeffs)
{
  std::vector<cv::Point3f> objectPoints(4);
  objectPoints[0] = cv::Vec3f(-fiducialSize / 2.f, fiducialSize / 2.f, 0);
  objectPoints[1] = cv::Vec3f(fiducialSize / 2.f, fiducialSize / 2.f, 0);
  objectPoints[2] = cv::Vec3f(fiducialSize / 2.f, -fiducialSize / 2.f, 0);
  objectPoints[3] = cv::Vec3f(-fiducialSize / 2.f, -fiducialSize / 2.f, 0);

  std::vector<cv::Point2f> projectedPoints;

  cv::projectPoints(objectPoints, marker.Rvec, marker.Tvec, cameraMatrix,
                    distCoeffs, projectedPoints);

  // calculate RMS image error
  double totalError = 0.0;
  for (unsigned int i = 0; i < objectPoints.size(); i++) {
    double error = dist(marker[i], projectedPoints[i]);
    totalError += error * error;
  }
  double rerror = totalError / (double) objectPoints.size();
  return rerror;
}
