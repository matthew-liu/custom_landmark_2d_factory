#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

#include <custom_landmark_2d/demor.h>

using namespace sensor_msgs;
using namespace message_filters;

// run this class like this:
// rosrun custom_landmark_2d demo template.jpg rgb:=/head_camera/rgb/image_raw depth:=/head_camera/depth_registered/image_raw
int main(int argc, char** argv) {

  ros::init(argc, argv, "demo");
  ros::NodeHandle nh;

  // Load template from stdin
  cv::Mat templ = cv::imread( argv[1], 1 );

  if ( !templ.data ) {
      ROS_ERROR("No template data \n");
      return -1;
  }

  custom_landmark_2d::Demor demor;

  // fetch CameraInfo
  demor.camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/head_camera/rgb/camera_info");

  ROS_INFO("received camear_info...");  

  // setup the matcher
  demor.matcher.set_template(templ);
  demor.matcher.set_camera_model(demor.camera_info);

  // fetch tf transform (only for dispaly purpose in Rviz, not necessary for the algorithm!)
  tf::TransformListener tf_listener;
  tf_listener.waitForTransform("base_link", "head_camera_rgb_optical_frame",
                               ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform;

  try {     
    tf_listener.lookupTransform("base_link", "head_camera_rgb_optical_frame",
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    std::cerr << e.what() << std::endl;
    ros::shutdown(); 
  } catch (tf::ExtrapolationException& e) {
    std::cerr << e.what() << std::endl;
    ros::shutdown(); 
  }
  // setup tf transform
  demor.transf = transform;

  // setup matchPointCloudsCallback
  demor.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("generated_cloud", 1, true);

  message_filters::Subscriber<Image> rgb_sub(nh, "rgb", 1); // rgb:=/head_camera/rgb/image_raw
  message_filters::Subscriber<Image> depth_sub(nh, "depth", 1); // depth:=/head_camera/depth_registered/image_raw

  typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&custom_landmark_2d::Demor::matchPointCloudsCallback, &demor, _1, _2));

  // setup matchCallback
  demor.matched_scene_pub = nh.advertise<sensor_msgs::Image>("image_out", 1000);
  ros::Subscriber sub = nh.subscribe("rgb", 5, &custom_landmark_2d::Demor::matchCallback, &demor);
  
  ros::spin();

  return 0;
}