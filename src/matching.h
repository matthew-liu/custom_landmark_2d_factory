#ifndef MATCHING_H
#define MATCHING_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>

using namespace std;
using namespace cv;

int match_method = CV_TM_CCOEFF_NORMED;
// max #times allowed for resizing
int counter = 3;
// the threshold for BEST acceptable matching points
float raw_match_limit = 0.7;
// the threshold for ALL acceptable matching points
float match_limit = 0.63;

int x_dist, y_dist;


double matching_method(Mat& img, Mat& templ, list<Point>& matching);
bool around(int x, int y, Point& p);
bool around_matching_points(int x, int y, list<Point>& matching, Point** point);


bool matching(Mat& img, Mat& templ, list<Point>& lst) {

  double first_value = matching_method(img, templ, lst);
  // success without rescaling
  if (first_value > raw_match_limit) return true;

  Mat scaled_templ;

  double value;
  
  // scale down
  resize(templ, scaled_templ, Size(), 0.9, 0.9, INTER_AREA);
  value = matching_method(img, scaled_templ, lst);

  if (value > raw_match_limit) return true;
  if (value > first_value) {
    double factor = 0.8;
    while (counter > 0) {
      resize(templ, scaled_templ, Size(), factor, factor, INTER_AREA);
      value = matching_method(img, scaled_templ, lst);
      if (value > raw_match_limit) return true;
      factor -= 0.1;
      counter--;
    }
  } else {
    // scale up
    resize(templ, scaled_templ, Size(), 1.1, 1.1, INTER_LINEAR);
    value = matching_method(img, scaled_templ, lst);
    if (value > raw_match_limit) return true;
    if (value > first_value) {
      double factor = 1.2;
      while (counter > 0) {
        resize(templ, scaled_templ, Size(), factor, factor, INTER_AREA);
        value = matching_method(img, scaled_templ, lst);
        if (value > raw_match_limit) return true;
        factor += 0.1;
        counter--;
      }
    }
  }

  return false;
}


double matching_method(Mat& img, Mat& templ, list<Point>& matching) {

  y_dist = (int) templ.rows;
  x_dist = (int) templ.cols;

  printf("\n--------\n");
  printf("x_dist: %d, y_dist: %d\n\n", x_dist, y_dist);

  // Create the result matrix
  Mat result;

  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  // Do the Matching
  matchTemplate( img, templ, result, match_method );

  double minVal; double maxVal; Point minLoc; Point maxLoc;
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

  printf("best match value: %f, position: %d, %d\n", result.at<float>(maxLoc), maxLoc.x, maxLoc.y);

  if (maxVal < raw_match_limit) {
    return maxVal;
  }

  matching.push_back(maxLoc);

  // scan through result to find all matching points
  int i,j;
  float* p;
  for( i = 0; i < result.rows; i++) {
      p = result.ptr<float>(i);
      for ( j = 0; j < result.cols; j++) {
          if ( p[j] > match_limit) { // acceptable matching point
            Point* point_ptr;
            if (around_matching_points(j, i, matching, &point_ptr)) {
              if (p[j] > result.at<float>(*point_ptr)) { // current point has better score
                point_ptr->x = j;
                point_ptr->y = i;
              }
            } else {
              matching.push_back(Point(j, i));
            }
          }
      }
  }


  printf("--------------\nMatched Points Info:\n\n");


  // display matched parts
  for (list<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
    printf("point intensity: %f, position: %d, %d\n", result.at<float>(*it), it->x, it->y);

    rectangle( img, *it, Point( it->x + templ.cols , it->y + templ.rows ), Scalar(255, 255, 0), 5, 8, 0 );
    rectangle( result, *it, Point( it->x + templ.cols , it->y + templ.rows ), Scalar(255, 255, 0), 5, 8, 0 );
  }
  printf("\n");

  return maxVal;
}


// check whether point(x, y) is around point p
bool around(int x, int y, Point& p) {
  if (abs(p.x - x) < x_dist && abs(p.y - y) < y_dist)
    return true;
  return false;
}


// check whether point(x, y) is around ANY point in the matching list
bool around_matching_points(int x, int y, list<Point>& matching, Point** point) {
  if (matching.empty())
    return false;

  for (list<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
    if (around(x, y, *it)) {
      *point = &(*it);
      return true;
    }
  }
  return false;
}

#endif // MATCHING_H