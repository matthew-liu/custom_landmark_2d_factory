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

  ros::Subscriber sub = n.subscribe("scene", 5, demo); // scene:=/head_camera/rgb/image_raw


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
    vector<custom_landmark_2d::Frame> lst;

    matcher.set_template(templ);
    bool result = matcher.match(cv_ptr->image, lst);

    if (result) {

      ROS_INFO("--------------\nMatched Points Info:\n\n");
      ROS_INFO("#matched objects: %lu\n", lst.size()); 

      // loop through the vector
      for (vector<custom_landmark_2d::Frame>::iterator it = lst.begin(); it != lst.end(); it++) {
        // annotates matched parts on scene
        rectangle( cv_ptr->image, it->p1, it->p2, Scalar(255, 255, 0), 5, 8, 0 );
        
        ROS_INFO("point intensity: %f, position: [%d, %d]\n", it->score, it->p1.x, it->p1.y);
      }
    }
    
    pub->publish(cv_ptr->toImageMsg());
  }
}
