#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>
#include <stdio.h>

#include <custom_landmark_2d/rgbd_pointcloud.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace sensor_msgs;

namespace custom_landmark_2d {

RgbdPointCloud::RgbdPointCloud(const CameraInfoConstPtr& camera_info) {
	cam_model.fromCameraInfo(camera_info);
};

bool RgbdPointCloud::to_pointcloud(const cv::Mat& rgb, const cv::Mat& depth, PointCloudC::Ptr cloud) {

	printf("CameraInfo frame: %s\n", cam_model.tfFrame().c_str());

	if (rgb.cols != depth.cols || rgb.rows != depth.rows) {
		printf("ERROR: depth image dimension doesn't match rgb image");
		return false;
	}

	cloud->clear();

	for( int i = 0; i < depth.rows; i++) {
		for ( int j = 0; j < depth.cols; j++) {
			float dist = depth.at<float>(i, j);
			cv::Vec3b color = rgb.at<cv::Vec3b>(i, j);
			if (!isnan(dist)) {
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

	cloud->width = (int) cloud->points.size();
	cloud->height = 1;

	return true;
}

}