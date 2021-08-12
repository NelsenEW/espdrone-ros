#include "image_transport_republisher.hpp"

image_transport::Publisher img_pub;

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_transport_republisher");
  ros::NodeHandle nh;

  ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("camera_stream", 1, cameraStreamCallback);
  image_transport::ImageTransport it(nh);
  img_pub = it.advertise("image_raw", 1);

  ros::spin();
}

void cameraStreamCallback(const sensor_msgs::ImageConstPtr& msg) {
  img_pub.publish(msg);
}
