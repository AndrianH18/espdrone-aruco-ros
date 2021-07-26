#include "image_transport_republisher.hpp"

image_transport::Publisher img_pub;

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_transport_republisher");
  ros::NodeHandle nh("~");

  std::string drone_name = nh.param<std::string>("drone_name", "esp_drone");
  ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/"+drone_name+"/camera_stream", 1, cameraStreamCallback);

  image_transport::ImageTransport it(nh);
  img_pub = it.advertise("/"+drone_name, 1);

  ros::spin();
}

void cameraStreamCallback(const sensor_msgs::ImageConstPtr& msg) {
  img_pub.publish(msg);
}
