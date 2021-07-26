#include <iostream>
#include <fstream>
#include <sstream>

#include <aruco/aruco.h>
#include <aruco/posetracker.h>
#include <aruco/cvdrawingutils.h>
#include <aruco_ros/aruco_ros_utils.hpp>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace aruco;
using namespace std;

class ArucoMap
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  aruco::MarkerDetector::Params mParams;

  std::string camera_frame;
  std::string map_frame;

  bool useRectifiedImages;
  bool draw_markers_;
  bool draw_markers_cube_;
  bool draw_markers_axis_;
  bool draw_map_axis_;
  bool publish_tf_;
  bool rotate_marker_axis_;

  MarkerDetector mDetector;
  vector<Marker> markers;
  vector<int> markers_from_map;

  MarkerMap mapConfig;
  MarkerMapPoseTracker mapPoseTracker;

  bool cam_info_received;
  ros::Subscriber cam_info_sub;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;

  double marker_size;
  std::string map_config_file;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf2_ros::Buffer _tfBuffer;
  tf2_ros::TransformListener _tfListener;
  tf2_ros::TransformBroadcaster br;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  ArucoMap()
    : cam_info_received(false),
      nh("~"),
      it(nh),
      _tfListener(_tfBuffer)
  {

    std::string refinementMethod;
    bool use_enclosed;
    nh.param<std::string>("corner_refinement", refinementMethod, "CORNER_LINES");
    nh.param<bool>("use_enclosed", use_enclosed, false);

    mParams.getCornerRefinementMethodFromString(refinementMethod);
    mParams.detectEnclosedMarkers(use_enclosed);

    ROS_INFO_STREAM("Corner refinement method: " << mParams.cornerRefinementM);
    ROS_INFO_STREAM("Threshold method: " << mParams.thresMethod);

    mDetector.setParameters(mParams);

    image_sub = it.subscribe("/image", 1, &ArucoMap::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoMap::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<std::string>("map_config", map_config_file, "board.yml");
    nh.param<std::string>("map_frame", map_frame, "marker_map");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    nh.param<bool>("draw_markers", draw_markers_, false);
    nh.param<bool>("draw_markers_cube", draw_markers_cube_, false);
    nh.param<bool>("draw_markers_axis", draw_markers_axis_, false);
    nh.param<bool>("draw_map_axis", draw_map_axis_, false);
    nh.param<bool>("publish_tf", publish_tf_, false);
    nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, false);

    try {
      mapConfig.readFromFile(map_config_file);
    } catch (std::exception & exp) {
      ROS_ERROR("Could not read '%s'", map_config_file.c_str());
      exit(1);
    }

    mDetector.setDictionary(mapConfig.getDictionary());

    ROS_ASSERT(!camera_frame.empty() && !map_frame.empty());

    ROS_INFO("Aruco node started with marker size of %f m and marker map to track: %s",
             marker_size, map_config_file.c_str());
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             camera_frame.c_str(), map_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoMap::reconf_callback, this, _1, _2));
  }

  void image_callback(const sensor_msgs::ImageConstPtr & msg)
  {
    if ((image_pub.getNumSubscribers() == 0) &&
      (debug_pub.getNumSubscribers() == 0) &&
      (pose_pub.getNumSubscribers() == 0) &&
      (transform_pub.getNumSubscribers() == 0) &&
      (position_pub.getNumSubscribers() == 0)) {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    if (!cam_info_received) return;

    ros::Time curr_stamp(ros::Time::now());
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      markers = mDetector.detect(inImage, camParam, marker_size);
      markers_from_map = mapConfig.getIndices(markers);

      if (mapConfig.isExpressedInMeters() && camParam.isValid()) {
        if (mapPoseTracker.estimatePose(markers)) {
          if (draw_map_axis_)
            aruco::CvDrawingUtils::draw3dAxis(inImage,
                                              camParam,
                                              mapPoseTracker.getRvec(),
                                              mapPoseTracker.getTvec(),
                                              mapConfig[0].getMarkerSize() * 2);

          tf2::Transform
            transform = aruco_ros::arucoMarkerMap2Tf(mapPoseTracker, rotate_marker_axis_);

          geometry_msgs::TransformStamped stampedTransform;
          stampedTransform.header.stamp = curr_stamp;
          stampedTransform.header.frame_id = camera_frame;
          stampedTransform.child_frame_id = map_frame;
          stampedTransform.transform = tf2::toMsg(transform);

          if (publish_tf_)
            br.sendTransform(stampedTransform);

          //publish for easier debugging
          transform_pub.publish(stampedTransform);

          geometry_msgs::PoseStamped poseMsg;
          tf2::convert(stampedTransform, poseMsg);
          pose_pub.publish(poseMsg);

          geometry_msgs::Vector3Stamped positionMsg;
          positionMsg.header = stampedTransform.header;
          positionMsg.vector = stampedTransform.transform.translation;
          position_pub.publish(positionMsg);
        }
      }

      for (size_t i = 0; draw_markers_ && i < markers.size(); ++i) {
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }

      if (camParam.isValid() && marker_size > 0) {
        for (size_t i = 0; i < markers.size(); ++i) {
          if (draw_markers_axis_)
            CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          if (draw_markers_cube_)
            CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
        }
      }

      if (image_pub.getNumSubscribers() > 0) {
        //show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0) {
        //show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.frame_id = msg->header.frame_id;
        debug_msg.header.stamp = msg->header.stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }
    } catch (cv_bridge::Exception & e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo & msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    mapPoseTracker.setParams(camParam, mapConfig);
    cam_info_received = true;
    cam_info_sub.shutdown();
    ROS_INFO("Received camera info");
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig & config, uint32_t level)
  {
    ROS_INFO("In dynamic reconfigure callback");
    mParams.setCornerRefinementMethod(static_cast<aruco::CornerRefinementMethod>(config.corner_refinement_method));
    mParams.setDetectionMode(static_cast<aruco::DetectionMode>(config.detection_mode),
                             config.min_marker_size);
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "aruco_single_board");
  ArucoMap node;
  ros::spin();
}
