#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "matching.h"

int main( int argc, char** argv ) {
  // Load image and template
  Mat img = imread( argv[1], 1 );
  Mat templ = imread( argv[2], 1 );

  if ( !img.data ) {
      printf("No image data \n");
      return -1;
  }
  if ( !templ.data ) {
      printf("No template data \n");
      return -1;
  }

  list<Point> lst;

  if (matching(img, templ, lst)) {
    printf("\n\nmatching success! \n");
  }


  printf("--------------\n REPEAT Matched Points Info:\n\n");
  printf("coordinates: %d, %d\n", x_dist, y_dist);
  // display matched parts
  for (list<Point>::iterator it = lst.begin(); it != lst.end(); it++) {
    printf("position: %d, %d\n", it->x, it->y);
  }

  char const * image_window = "Source Image";

  namedWindow(image_window, CV_WINDOW_NORMAL);
  resizeWindow(image_window, 600,600);

  imshow( image_window, img );

  waitKey(0);
}
