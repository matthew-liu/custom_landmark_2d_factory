#include <custom_landmark_2d/matcher.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

#include <stdlib.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace custom_landmark_2d {

Frame::Frame(const Point p1, const Point p2, const float score) : p1(p1), p2(p2), score(score) {}

Matcher::Matcher() : match_method(CV_TM_CCOEFF_NORMED),
					 count_times(2),
					 raw_match_limit(0.75),
					 match_limit(0.68) {}

void Matcher::set_template(const Mat& templ) {
	this->templ = templ;
}

void Matcher::set_camera_model(const sensor_msgs::CameraInfoConstPtr& camera_info) {
	cam_model.fromCameraInfo(camera_info);
}

bool Matcher::match_pointclouds(const Mat& rgb, const Mat& depth, vector<PointCloudC::Ptr>& object_clouds) {

	// printf("CameraInfo frame: %s\n", cam_model.tfFrame().c_str());

	if (rgb.cols != depth.cols || rgb.rows != depth.rows) {
		return false;
	}

	vector<Frame> lst;

    if (!match(rgb, lst)) return false; // no match found

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
					Frame* f = &lst[k];
					if (j >= f->p1.x && j <= f->p2.x && i >= f->p1.y && i <= f->p2.y) {
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

bool Matcher::match(const Mat& scene, std::vector<Frame>& lst) {
	
	lst.clear();

	int counter = count_times;
	Mat scaled_templ;

	// scale up
	double factor = 1.0 + 0.1 * count_times;
	while (counter > 0) {

		resize(templ, scaled_templ, Size(), factor, factor, INTER_LINEAR);
		exact_match(scene, scaled_templ, lst);

		factor -= 0.1;
		counter--;
	}

	// original scale
	exact_match(scene, templ, lst);

	// scale down
	counter = count_times;
	factor  = 0.9;
	while (counter > 0) {

		resize(templ, scaled_templ, Size(), factor, factor, INTER_AREA);
		exact_match(scene, scaled_templ, lst);

		factor -= 0.1;
		counter--;
	}

	if (lst.empty()) {
		return false;
	}

	return true;
}

// performs a single match on the given scene & templ, returns the max match_score	
double Matcher::exact_match(const Mat& scene, const Mat& templ, vector<Frame>& matching) {

	x_dist = (int) templ.cols;
	y_dist = (int) templ.rows;

	Mat result; // the result matrix

	int result_cols = scene.cols - templ.cols + 1;
	int result_rows = scene.rows - templ.rows + 1;

	result.create(result_rows, result_cols, CV_32FC1);

	// Do the Matching
	matchTemplate(scene, templ, result, match_method);

	double minVal; double maxVal; Point minLoc; Point maxLoc;
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	if (maxVal < raw_match_limit) {
		return maxVal;
	}

	// scan through result to find all matching points
	int i,j;
	float* p;
	for( i = 0; i < result.rows; i++) {
		p = result.ptr<float>(i);
		for ( j = 0; j < result.cols; j++) {
			if ( p[j] > match_limit) { // acceptable matching point
				Frame* f_ptr;
				if (around_frame(j, i, matching, &f_ptr)) {
					if (p[j] > f_ptr->score) { // current point has better score

						f_ptr->p1 = Point(j, i);
						f_ptr->p2 = Point(j + x_dist, i + y_dist);
            			f_ptr->score = p[j];
          			}
        		} else {
          			matching.push_back(Frame(Point(j, i), Point(j + x_dist, i + y_dist), p[j]));
        		}
      		}
  		}
	}

	return maxVal;
}

// checks whether point(x, y) is around any frame in the vector, returns such frame if found
bool Matcher::around_frame(int x, int y, vector<Frame>& matching, Frame** f_ptr_ptr) {
	if (matching.empty()) {
		return false;
	}

	for (vector<Frame>::iterator it = matching.begin(); it != matching.end(); it++) {
		if (around_point(x, y, it->p1)) {
 			*f_ptr_ptr = &(*it);
  			return true;
		}
	}
	return false;
}

// checks whether point(x, y) is around point p using the current x_dist & y_dist
bool Matcher::around_point(int x, int y, Point& p) {
	if (abs(p.x - x) < x_dist && abs(p.y - y) < y_dist) {
    	return true;
	}
  	return false;
}

}