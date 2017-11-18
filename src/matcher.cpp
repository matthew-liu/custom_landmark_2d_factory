#include "custom_landmark_2d/matcher.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>

using namespace std;
using namespace cv;

namespace custom_landmark_2d {

Matcher::Matcher() : match_method(CV_TM_CCOEFF_NORMED),
					 count_times(3),
					 raw_match_limit(0.75),
					 match_limit(0.68) {}

bool Matcher::scale_match(const cv::Mat& scene, const cv::Mat& templ,
                          std::list<cv::Point>& lst,
				 		  int* width, int* height) {
	
	int counter = count_times;

	printf("---trial origin---\n");
	double unscaled_value = match(scene, templ, lst);

	// success without rescaling
	if (unscaled_value > raw_match_limit) {
		*width = x_dist;
		*height = y_dist;
		return true;
	}

	Mat scaled_templ;

	double value;
	double factor = 0.9;

	// scale down
	printf("---trial downsize 1---\n");
	resize(templ, scaled_templ, Size(), factor, factor, INTER_AREA);
	value = match(scene, scaled_templ, lst);

	if (value > raw_match_limit) {
		*width = x_dist;
		*height = y_dist;
		return true;
	}

	if (value > unscaled_value) {
		counter--; // already scaled down once
		while (counter > 0) {
			factor -= 0.1; // 0.8-
			counter--;
			printf("---trial downsize %d---\n", count_times - counter);
			resize(templ, scaled_templ, Size(), factor, factor, INTER_AREA);
			value = match(scene, scaled_templ, lst);

			if (value > raw_match_limit) {
				*width = x_dist;
				*height = y_dist;
				return true;
			}
		}
	} else {
		factor = 1.0;
		// scale up
  		while (counter > 0) {
  			factor += 0.1; // 1.1+
		    counter--;
		    printf("---trial upsize %d---\n", count_times - counter);
		    resize(templ, scaled_templ, Size(), factor, factor, INTER_LINEAR);
		    value = match(scene, scaled_templ, lst);

		    if (value > raw_match_limit) {
				*width = x_dist;
				*height = y_dist;
		    	return true;
		    }
  		}
	}

	return false;
}

// performs a single match on the given scene & templ, returns the max match_score	
double Matcher::match(const Mat& scene, const Mat& templ, list<cv::Point>& matching) {

	x_dist = (int) templ.cols;
	y_dist = (int) templ.rows;

	printf("x_dist: %d, y_dist: %d\n\n", x_dist, y_dist);

	Mat result; // the result matrix

	int result_cols = scene.cols - templ.cols + 1;
	int result_rows = scene.rows - templ.rows + 1;

	result.create(result_rows, result_cols, CV_32FC1);

	// Do the Matching
	matchTemplate(scene, templ, result, match_method);

	double minVal; double maxVal; Point minLoc; Point maxLoc;
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

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
				if (around_points(j, i, matching, &point_ptr)) {
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

	// annotates matched parts on scene
	for (list<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
		printf("point intensity: %f, position: %d, %d\n", result.at<float>(*it), it->x, it->y);
	}
	printf("\n");

	return maxVal;
}

// checks whether point(x, y) is around any point p in the list, returns such p if found
bool Matcher::around_points(int x, int y, list<cv::Point>& matching, Point** p_ptr_ptr) {
	if (matching.empty())
		return false;

	for (list<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
		if (around_point(x, y, *it)) {
 			*p_ptr_ptr = &(*it);
  			return true;
		}
	}
	return false;
}

// checks whether point(x, y) is around point p
bool Matcher::around_point(int x, int y, Point& p) {
	if (abs(p.x - x) < x_dist && abs(p.y - y) < y_dist)
    	return true;
  	return false;
}

}