#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

namespace custom_landmark_2d {

class RgbdPointCloud {

	public:
		RgbdPointCloud(const sensor_msgs::CameraInfoConstPtr& camera_info);
		bool to_pointcloud(const cv::Mat& rgb, const cv::Mat& depth, 
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	private:
		image_geometry::PinholeCameraModel cam_model;
};

}