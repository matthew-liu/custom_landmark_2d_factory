#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <custom_landmark_2d/matcher.h>

namespace custom_landmark_2d {

class Demor {
	public:
		Matcher matcher;
		tf::StampedTransform transf; // tf transf for pointCloud display
		sensor_msgs::CameraInfoConstPtr camera_info;

		ros::Publisher cloud_pub;
		ros::Publisher matched_scene_pub;

		void matchPointCloudsCallback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);
		void matchCallback(const sensor_msgs::Image::ConstPtr& msg);
};

}