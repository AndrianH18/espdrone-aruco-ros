#include <mutex>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/PixelToPosition.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat in_image_;
  aruco::CameraParameters cam_param_;
  aruco::MarkerDetector::Params detector_params_;
  MarkerDetector detector_;
  std::map<int, MarkerPoseTracker> tracker_;
  std::vector<Marker> markers_;

  std::string camera_frame_;
  std::string reference_frame_;
  std::mutex marker_mutex_;

  std::string dictionary_;
  double marker_size_;
  std::map<int, double> marker_sizes_;

  bool cam_info_received_;
  bool rotate_marker_axis_;
  bool use_pose_tracker_;
  bool draw_markers_;
  bool use_rectified_images_;

  aruco_msgs::MarkerArray marker_msg_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber cam_info_sub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer pixels_server_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server_;

public:
  ArucoSimple()
  : marker_size_(-1),
    cam_info_received_(false),
    rotate_marker_axis_(false),
    use_pose_tracker_(true),
    draw_markers_(true),
    use_rectified_images_(true),
    nh_("~"),
    it_(nh_),
    tf_listener_(tf_buffer_)
  {
    // set detector_ parameters
    std::string refinementMethod;
    nh_.param<std::string>("corner_refinement", refinementMethod, "CORNER_LINES");
    aruco::MarkerDetector::Params::getCornerRefinementMethodFromString(refinementMethod);

    ROS_INFO_STREAM("Corner refinement method: " << detector_params_.cornerRefinementM);
    ROS_INFO_STREAM("Threshold method: " << detector_params_.thresMethod);

    detector_.setParameters(detector_params_);

    nh_.param<double>("marker_size", marker_size_, 0.041);
    nh_.param<std::string>("dictionary", dictionary_, "ARUCO");
    nh_.param<bool>("image_is_rectified", use_rectified_images_, true);
    nh_.param<bool>("rotate_marker_axis", rotate_marker_axis_, false);
    nh_.param<bool>("use_pose_tracker", use_pose_tracker_, true);
    nh_.param<bool>("draw_markers", draw_markers_, true);
    nh_.param<std::string>("reference_frame", reference_frame_, "");
    readMarkerSizeParam();

    detector_.setDictionary(dictionary_);

    if (draw_markers_) {
      image_pub_ = it_.advertise("result", 1);
    }

    marker_pub_ = nh_.advertise<aruco_msgs::MarkerArray>("markers", 1);

    pixels_server_ = nh_.advertiseService("pixel_to_position",
                                          &ArucoSimple::pixelToPositionCallback, this);

    ROS_WARN("Aruco node is ready!");

    image_sub_ = it_.subscribe("/image", 1, &ArucoSimple::imageCallback, this);
    cam_info_sub_ = nh_.subscribe("/camera_info", 1, &ArucoSimple::camInfoCallback, this);

    dyn_rec_server_.setCallback(boost::bind(&ArucoSimple::reconfCallback, this, _1, _2));
  }

  void readMarkerSizeParam()
  {
    std::string str;
    std::vector<std::string> strs;

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    nh_.param<std::string>("marker_size_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const std::string & element : strs) {
      if (element.empty()) {
        continue;
      }
      std::vector<std::string> parts;
      boost::split(parts, element, boost::is_any_of(":"));
      if (parts.size() == 2) {
        double len = std::stod(parts[1]);
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
          int start = std::stoi(range[0]);
          int end = std::stoi(range[1]);
          ROS_INFO("Setting fiducial id range %d - %d length to %f",
                   start, end, len);
          for (int j = start; j <= end; j++) {
            marker_sizes_[j] = len;
          }
        } else if (range.size() == 1) {
          int fid = std::stoi(range[0]);
          ROS_INFO("Setting fiducial id %d length to %f", fid, len);
          marker_sizes_[fid] = len;
        } else {
          ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
      } else {
        ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
      }
    }
  }

  void imageCallback(const sensor_msgs::ImageConstPtr & msg)
  {
    if (cam_info_received_) {

      ros::Time curr_stamp(ros::Time::now());
      cv_bridge::CvImagePtr cv_ptr;

      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        in_image_ = cv_ptr->image;

        // detection results will go into "markers_"
        {
          std::lock_guard<std::mutex> lock(marker_mutex_);
          detector_.detect(in_image_, markers_, cam_param_);
        }

        marker_msg_.header.stamp = curr_stamp;
        marker_msg_.header.frame_id = reference_frame_;
        marker_msg_.markers.clear();

        aruco_msgs::Marker m;
        float fiducial_size;

        // for each marker, draw info and its boundaries in the image
        for (auto & marker : markers_) {

          // find size of the marker to use
          fiducial_size = (float) marker_size_;
          auto it = marker_sizes_.find(marker.id);
          if (it != marker_sizes_.end()) {
            fiducial_size = it->second;
          }

          // if marker pose tracker is enabled, use it to calculate extrinsics
          if (use_pose_tracker_) {
            if (!tracker_[marker.id].estimatePose(marker, cam_param_, fiducial_size)) {
              ROS_WARN("Failed to detect pose for ID: %d", marker.id);
              continue;
            }
          }

          // if marker pose tracker is disabled or has failed earlier
          // calculate extrinsics normally
          if (!use_pose_tracker_ || marker.Tvec.empty() || marker.Rvec.empty()) {
            marker.calculateExtrinsics(fiducial_size, cam_param_, false);
          }

          // if extrinsics can still not be calculated, just draw the markers and continue
          if (marker.Tvec.empty() || marker.Rvec.empty()) {
            ROS_WARN("Failed to detect pose for ID: %d", marker.id);
            marker.draw(in_image_, cv::Scalar(0, 0, 255), 2);
            continue;
          }

          // get proper transform
          tf2::Transform transform = aruco_ros::arucoMarker2Tf(marker, rotate_marker_axis_);

          tf2::Transform cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame_ != camera_frame_) {
            getTransform(reference_frame_, camera_frame_, cameraToReference);
          }

          transform = cameraToReference * transform;

          // fill marker msg
          m.header.stamp = curr_stamp;
          m.header.frame_id = reference_frame_;
          m.id = marker.id;
          m.center.x = marker.getCenter().x;
          m.center.y = marker.getCenter().y;
          m.transform = tf2::toMsg(transform);

          m.fiducial_area = aruco_ros::getFiducialArea(marker);
          m.image_error = aruco_ros::getReprojectionError(
            fiducial_size, marker, cam_param_.CameraMatrix, cam_param_.Distorsion);

          // Convert image_error (in pixels) to object_error (in meters)
          m.object_error =
            (m.image_error / aruco_ros::dist(marker[0], marker[2])) *
              (norm(marker.Tvec) / fiducial_size);

          // add marker to array to publish
          marker_msg_.markers.push_back(m);

          // draw marker on image
          if (draw_markers_) {
            marker.draw(in_image_, cv::Scalar(0, 0, 255), 2);
            CvDrawingUtils::draw3dAxis(in_image_, marker, cam_param_);
          }
        }

        // publish marker array
        if (!markers_.empty()) {
          marker_pub_.publish(marker_msg_);
        }

        if (image_pub_.getNumSubscribers() > 0 && draw_markers_) {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = in_image_;
          image_pub_.publish(out_msg.toImageMsg());
        }
      }
      catch (...) {
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void camInfoCallback(const sensor_msgs::CameraInfo & msg)
  {
    cam_param_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, use_rectified_images_);

    try {
      if (!cam_param_.isValid()) {
        ROS_WARN("Camera info was invalid, waiting for a valid message!");
        return;
      }
    } catch (...) {
      ROS_WARN("Error while reading camera info, waiting for a valid message!");
      return;
    }

    camera_frame_ = msg.header.frame_id;
    if (reference_frame_.empty()) {
      reference_frame_ = camera_frame_;
    }

    ROS_WARN(
      "Received camera info, camera frame was set to '%s'. "
      "Marker pose will be published in '%s' frame.",
      camera_frame_.c_str(), reference_frame_.c_str());

    cam_info_received_ = true;
    cam_info_sub_.shutdown();
  }

  // shifts the last found marker's center to the pixel requested
  // and calculate extrinsics
  bool pixelToPositionCallback(
    aruco_msgs::PixelToPosition::Request & req,
    aruco_msgs::PixelToPosition::Response & resp)
  {
    std::lock_guard<std::mutex> lock(marker_mutex_);

    if (markers_.empty() || !cam_info_received_) {
      return false;
    }

    for (auto & marker : markers_) {

      if (marker.id == req.id) {

        resp.positions.resize(req.pixels.size());
        Marker tmp_marker;

        for (size_t i = 0; i < req.pixels.size(); ++i) {
          marker.copyTo(tmp_marker);
          double diff_x = marker.getCenter().x - req.pixels[i].x;
          double diff_y = marker.getCenter().y - req.pixels[i].y;

          for (int idx = 0; idx < tmp_marker.size(); ++idx) {
            tmp_marker[idx].x -= diff_x;
            tmp_marker[idx].y -= diff_y;
          }

          // find size of the marker to use
          auto fiducial_size = (float) marker_size_;
          auto it = marker_sizes_.find(marker.id);
          if (it != marker_sizes_.end()) {
            fiducial_size = it->second;
          }

          tmp_marker.calculateExtrinsics(fiducial_size, cam_param_, false);

          if (tmp_marker.isPoseValid()) {
            tf2::Transform transform = aruco_ros::arucoMarker2Tf(tmp_marker, rotate_marker_axis_);
            resp.positions[i].x = transform.getOrigin().x();
            resp.positions[i].y = transform.getOrigin().y();
            resp.positions[i].z = transform.getOrigin().z();
          } else {
            return false;
          }
        }

        resp.header.stamp = ros::Time::now();
        resp.header.frame_id = camera_frame_;
        return true;
      }
    }

    return false;
  }

  bool getTransform(const std::string & refFrame,
                    const std::string & childFrame,
                    tf2::Transform & transform)
  {
    std::string errMsg;

    if (!tf_buffer_.canTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), &errMsg)) {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    } else {
      try {
        auto tf = tf_buffer_.lookupTransform(refFrame, childFrame, ros::Time(0));
        tf2::fromMsg(tf.transform, transform);
      }
      catch (const tf2::TransformException & e) {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }
    }
    return true;
  }

  void reconfCallback(aruco_ros::ArucoThresholdConfig & config, uint32_t level)
  {
    ROS_INFO("In dynamic reconfigure callback");
    detector_params_.setCornerRefinementMethod(
      static_cast<aruco::CornerRefinementMethod>(config.corner_refinement_method));
    detector_params_.setDetectionMode(
      static_cast<aruco::DetectionMode>(config.detection_mode), config.min_marker_size);
  }

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aruco_simple");
  ArucoSimple node;
  ros::spin();
}
