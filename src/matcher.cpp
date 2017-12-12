#include <custom_landmark_2d/matcher.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace custom_landmark_2d {

Matcher::Matcher() : match_method(CV_TM_CCOEFF_NORMED),
					 count_times(3),
					 raw_match_limit(0.75),
					 match_limit(0.68) {}

Matcher::Matcher(const Mat& input_templ, const sensor_msgs::CameraInfoConstPtr& camera_info) : 
					 match_method(CV_TM_CCOEFF_NORMED),
					 count_times(3),
					 raw_match_limit(0.75),
					 match_limit(0.68),
					 templ(input_templ) {

	cam_model.fromCameraInfo(camera_info);
}

void Matcher::set_template(const Mat& input_templ) {
	templ = input_templ;
}

void Matcher::set_camera_info(const sensor_msgs::CameraInfoConstPtr& camera_info) {
	cam_model.fromCameraInfo(camera_info);
}

bool Matcher::match_pointclouds(const Mat& rgb, const Mat& depth, vector<PointCloudC::Ptr>& object_clouds) {

	printf("CameraInfo frame: %s\n", cam_model.tfFrame().c_str());

	if (rgb.cols != depth.cols || rgb.rows != depth.rows) {
		printf("ERROR: depth image dimension doesn't match rgb image");
		return false;
	}

	vector<Point> lst;
	int width;
    int height;

    if (!match(rgb, lst, &width, &height)) return false; // no match found

    object_clouds.clear();
    object_clouds.resize(lst.size());

    for( int i = 0; i < depth.rows; i++) {
		for ( int j = 0; j < depth.cols; j++) {
			float dist = depth.at<float>(i, j);
			if (!isnan(dist)) {
				// check if within any 2-d bounding box of matched objects:
				bool within_box = false;
				int c_index = -1;
				for (int k = 0; k < lst.size(); k++) {
					Point* p = &lst[k];
					if (j >= p->x && j <= p->x + width && i >= p->y && i <= p->y + height) {
						within_box = true;
						c_index = k;
						break;
					}
				}
				if (within_box) {
					cv::Vec3b color = rgb.at<cv::Vec3b>(i, j);
					cv::Point2d p_2d;
					p_2d.x = j;
					p_2d.y = i;

					cv::Point3d p_3d = cam_model.projectPixelTo3dRay(p_2d);

					PointC pcl_point;

					pcl_point.x = p_3d.x * dist;
					pcl_point.y = p_3d.y * dist;
					pcl_point.z = p_3d.z * dist;

					// bgr
					pcl_point.b = static_cast<uint8_t> (color[0]);
					pcl_point.g = static_cast<uint8_t> (color[1]);
					pcl_point.r = static_cast<uint8_t> (color[2]);

					if (!object_clouds[c_index]) {
						object_clouds[c_index] = PointCloudC::Ptr(new PointCloudC());
					}
					object_clouds[c_index]->points.push_back(pcl_point);	
				}
			}		
		}
	} 

	for (int i = 0; i < object_clouds.size(); i++) {
		object_clouds[i]->width = (int) object_clouds[i]->points.size();
		object_clouds[i]->height = 1;
	}

	return true;
}

bool Matcher::match_pointcloud(const Mat& rgb, const Mat& depth, PointCloudC::Ptr cloud) {

	printf("CameraInfo frame: %s\n", cam_model.tfFrame().c_str());

	if (rgb.cols != depth.cols || rgb.rows != depth.rows) {
		printf("ERROR: depth image dimension doesn't match rgb image");
		return false;
	}

	cloud->clear();

	vector<Point> lst;
	int width;
    int height;

    if (!match(rgb, lst, &width, &height)) return false; // no match found
 

	for( int i = 0; i < depth.rows; i++) {
		for ( int j = 0; j < depth.cols; j++) {
			float dist = depth.at<float>(i, j);
			if (!isnan(dist)) {
				// check if within any 2-d bounding box of matched objects:
				bool within_box = false;
				for (vector<Point>::iterator it = lst.begin(); it != lst.end(); it++) {
				// it->x + width , it->y + height
					if (j >= it->x && j <= it->x + width && i >= it->y && i <= it->y + height) {
						within_box = true;
						break;
					}
				}
				if (within_box) {
					cv::Vec3b color = rgb.at<cv::Vec3b>(i, j);
					cv::Point2d p_2d;
					p_2d.x = j;
					p_2d.y = i;

					cv::Point3d p_3d = cam_model.projectPixelTo3dRay(p_2d);

					PointC pcl_point;

					pcl_point.x = p_3d.x * dist;
					pcl_point.y = p_3d.y * dist;
					pcl_point.z = p_3d.z * dist;

					// bgr
					pcl_point.b = static_cast<uint8_t> (color[0]);
					pcl_point.g = static_cast<uint8_t> (color[1]);
					pcl_point.r = static_cast<uint8_t> (color[2]);

					cloud->points.push_back(pcl_point);	
				}
			}		
		}
	} 

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	return true;
}

bool Matcher::match(const Mat& scene, std::vector<Point>& lst, int* width, int* height) {

	// TODO: improve upscale effects
	
	int counter = count_times;

	printf("---trial origin---\n");
	double unscaled_value = exact_match(scene, templ, lst);

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
	value = exact_match(scene, scaled_templ, lst);

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
			value = exact_match(scene, scaled_templ, lst);

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
		    value = exact_match(scene, scaled_templ, lst);

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
double Matcher::exact_match(const Mat& scene, const Mat& templ, vector<Point>& matching) {

	// TODO: keep the points in matching to find the matching objects of all scales in the scene;
	// need to construct a new "point" object as well, with fields x, y, width, & height;
	// can use parallel threads to speed up the process.
	matching.clear(); 

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
	for (vector<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
		printf("point intensity: %f, position: %d, %d\n", result.at<float>(*it), it->x, it->y);
	}
	printf("\n");

	return maxVal;
}

// checks whether point(x, y) is around any point p in the vector, returns such p if found
bool Matcher::around_points(int x, int y, vector<Point>& matching, Point** p_ptr_ptr) {
	if (matching.empty())
		return false;

	for (vector<Point>::iterator it = matching.begin(); it != matching.end(); it++) {
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