#include <iostream>
#include <string>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& rgb, const ImageConstPtr& depth, std::string& filename) {
  ROS_INFO("received 2 images");

  // fetch CameraInfo
  sensor_msgs::CameraInfoConstPtr camera_info =
      ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
          "/head_camera/rgb/camera_info");

  ROS_INFO("received camear_info");  

  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  bag.write("/head_camera/rgb/image_raw", ros::Time::now(), rgb);
  bag.write("/head_camera/depth_registered/image_raw", ros::Time::now(), depth);
  bag.write("/head_camera/rgb/camera_info", ros::Time::now(), camera_info);
  bag.close();

  ros::shutdown(); 
}

void print_usage() {
  std::cout << "Usage: rosrun custom_landmark_2d save_2d_images_main FILENAME" << std::endl;
}

int main(int argc, char** argv) {
  
  if (argc < 2) {
    print_usage();
    return 1;
  }
  
  std::string filename(std::string(argv[1])  + ".bag");

  ros::init(argc, argv, "save_2d_images_main");

  ros::NodeHandle nh;  

  message_filters::Subscriber<Image> rgb_sub(nh, "/head_camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/head_camera/depth_registered/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, filename));

  ros::spin();

  return 0;
}