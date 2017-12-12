#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "custom_landmark_2d/Point.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "custom_landmark_2d/matcher.h"

using namespace std;
using namespace cv;

ros::Publisher* pub;
ros::Publisher* points_pub;
Mat templ;
custom_landmark_2d::Matcher matcher;

void demo(const sensor_msgs::Image::ConstPtr& msg);

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


  ros::Subscriber sub = n.subscribe("/head_camera/rgb/image_raw", 5, demo);


  ros::spin();


  return 0;
}

void demo(const sensor_msgs::Image::ConstPtr& msg) {
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
    vector<Point> lst;
    int width;
    int height;
    matcher.set_template(templ);
    bool result = matcher.match(cv_ptr->image, lst, &width, &height);

    if (result) {
      custom_landmark_2d::Point point;
      // loop through the vector
      for (vector<Point>::iterator it = lst.begin(); it != lst.end(); it++) {
        point.x = it->x;
        point.y = it->y;
        point.width = width;
        point.height = height;
        points_pub->publish(point);
        // annotates matched parts on scene
        rectangle( cv_ptr->image, *it, Point( it->x + width , it->y + height ), Scalar(255, 255, 0), 5, 8, 0 );
      }
    }
    

    pub->publish(cv_ptr->toImageMsg());
  }
}
