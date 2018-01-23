#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>

#include <time.h>

#include <custom_landmark_2d/demor.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace custom_landmark_2d {

void Demor::matchPointCloudsCallback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth) {

	// convert sensor_msgs::Images to cv::Mats
	cv_bridge::CvImagePtr rgb_ptr;
	cv_bridge::CvImagePtr depth_ptr;

	try {
		rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
		depth_ptr = cv_bridge::toCvCopy(depth); // 32FC1
	} 
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		ros::shutdown(); 
	}

	// the result pointCloud vector of matched objects
	std::vector<PointCloudC::Ptr> object_clouds;

	// run the method & count its runtime
	float start_tick = clock();
	bool result = matcher.match_pointclouds(rgb_ptr->image, depth_ptr->image, object_clouds);
	float end_tick = clock();
	ROS_INFO("runtime: %f\n", (end_tick - start_tick) / CLOCKS_PER_SEC); 

	if (!result) {
		ROS_INFO("no matched objects...\n");
		return; 
	}

	// the single pointCloud for better display 
	PointCloudC::Ptr pcl_cloud(new PointCloudC());

	ROS_INFO("#matched_objects: %lu\n", object_clouds.size()); 

	// concatenate the vector of pointClouds
	for (std::vector<PointCloudC::Ptr>::iterator it = object_clouds.begin(); it != object_clouds.end(); it++) {
		*pcl_cloud += **it;
	}
 
	ROS_INFO("pcl cloud size: %lu\n", pcl_cloud->size());  

	sensor_msgs::PointCloud2 ros_cloud;
	pcl::toROSMsg(*pcl_cloud, ros_cloud);
	ros_cloud.header.frame_id = camera_info->header.frame_id; // head_camera_rgb_optical_frame

	sensor_msgs::PointCloud2 cloud_out;
	// Transform a sensor_msgs::PointCloud2 dataset from its frame to a given TF target frame. 
	pcl_ros::transformPointCloud("base_link", transf, ros_cloud, cloud_out);

	cloud_pub.publish(cloud_out);
}

void Demor::matchCallback(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} 
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		exit(1);
	}

	// run matching on the image
	std::vector<custom_landmark_2d::Frame> lst;

	if (matcher.match(cv_ptr->image, lst)) {

		ROS_INFO("--------------\nMatched Points Info:\n\n");
		ROS_INFO("#matched objects: %lu\n", lst.size()); 

		// loop through the vector
		for (std::vector<custom_landmark_2d::Frame>::iterator it = lst.begin(); it != lst.end(); it++) {
			// annotates matched parts on scene
			rectangle( cv_ptr->image, it->p1, it->p2, cv::Scalar(255, 255, 0), 5, 8, 0 );

			ROS_INFO("point intensity: %f, position: [%d, %d]\n", it->score, it->p1.x, it->p1.y);
		}
	}

	matched_scene_pub.publish(cv_ptr->toImageMsg());
}

}