#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "matching.h"

ros::Publisher* pub;
Mat templ;

void matcher(const sensor_msgs::Image::ConstPtr& msg);

int main( int argc, char** argv ) {
  // Load template from stdin
  templ = imread( argv[1], 1 );

  if ( !templ.data ) {
      printf("No template data \n");
      return -1;
  }

  ros::init(argc, argv, "landmark_2d");

  ros::NodeHandle n;

  ros::Publisher pub_instance = n.advertise<sensor_msgs::Image>("output_channel", 1000);
  pub = &pub_instance;

  ros::Subscriber sub = n.subscribe("/head_mount_kinect/rgb/image_raw", 1000, matcher);

  // wait ~3 secs for each image processing
  ros::Rate loop_rate(0.3);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void matcher(const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO("scene img received sucessfully");

  if (ros::ok()) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // , sensor_msgs::image_encodings::RGB8
    } 
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      exit(1);
    }

    // run matching on the image
    list<Point> lst;
    bool result = matching(cv_ptr->image, templ, lst);

    if (!result) {
      ROS_INFO("--No matching for the template exists in current scene--\n");
      return;
    }
    
    // printf("--------------\n REPEAT Matched Points Info:\n\n");
    // printf("coordinates: %d, %d\n", x_dist, y_dist);
    // // display matched parts
    // for (list<Point>::iterator it = lst.begin(); it != lst.end(); it++) {
    //   printf("position: %d, %d\n", it->x, it->y);
    // }

    pub->publish(cv_ptr->toImageMsg());
  }
}
