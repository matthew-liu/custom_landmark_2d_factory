#include <iostream>
#include <string>

#include "ros/ros.h"
#include "rosbag/bag.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

using namespace sensor_msgs;
using namespace message_filters;

std::string* filename_ptr;

void callback(const ImageConstPtr& rgb, const ImageConstPtr& depth) {
  ROS_INFO("received 2 images");
  rosbag::Bag bag;
  bag.open(*filename_ptr, rosbag::bagmode::Write);
  bag.write("/head_camera/rgb/image_raw", ros::Time::now(), rgb);
  bag.write("/head_camera/depth_registered/image_raw", ros::Time::now(), depth);
  bag.close();

  std::exit(0);
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
  filename_ptr = &filename;

  ros::init(argc, argv, "save_2d_images_main");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> rgb_sub(nh, "/head_camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/head_camera/depth_registered/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}