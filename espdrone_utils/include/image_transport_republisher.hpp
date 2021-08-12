#ifndef IMAGE_TRANSPORT_REPUBLISHER_HPP_
#define IMAGE_TRANSPORT_REPUBLISHER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

void cameraStreamCallback(const sensor_msgs::ImageConstPtr& msg);

#endif  // IMAGE_TRANSPORT_REPUBLISHER_HPP_