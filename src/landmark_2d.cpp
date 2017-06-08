#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "custom_landmark_2d/Point.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "matching.h"

ros::Publisher* pub;
ros::Publisher* points_pub;
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

  ros::Publisher pub_instance = n.advertise<sensor_msgs::Image>("image_output", 1000);
  pub = &pub_instance;

  ros::Publisher points_pub_instance = n.advertise<custom_landmark_2d::Point>("points_output", 1000);
  points_pub = &points_pub_instance;


  ros::Subscriber sub = n.subscribe("/wide_stereo/right/image_color", 5, matcher);


  ros::spin();


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

    if (result) {
      custom_landmark_2d::Point point;
      // loop through the list
      for (list<Point>::iterator it = lst.begin(); it != lst.end(); it++) {
        point.x = it->x;
        point.y = it->y;
        point.width = x_dist;
        point.height = y_dist;
        points_pub->publish(point);
      }
    }
    

    pub->publish(cv_ptr->toImageMsg());
  }
}
