#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

namespace custom_landmark_2d {

class Matcher {

  public:
      int count_times; 		   // max #times allowed for resizing
      float raw_match_limit; // the threshold for BEST acceptable matching points
      float match_limit;     // the threshold for ALL acceptable matching points
      Matcher();
      Matcher(const cv::Mat& input_templ, const sensor_msgs::CameraInfoConstPtr& camera_info);

      void set_template(const cv::Mat& input_templ);
      void set_camera_info(const sensor_msgs::CameraInfoConstPtr& camera_info);
      // returns a vector of cv::Point that defines the upper-left corner of the 2-d bounding box
      // of matched templates in the scene, together with the width & height of the box.
      bool match(const cv::Mat& scene, std::list<cv::Point>& lst, int* width, int* height);

      bool match_pointcloud(const cv::Mat& rgb, const cv::Mat& depth, 
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  private:
      int match_method;
      int x_dist, y_dist;
      cv::Mat templ;
      image_geometry::PinholeCameraModel cam_model;

      // performs a single match on the given scene & templ, returns the max match_score	
      double exact_match(const cv::Mat& scene, const cv::Mat& templ, std::list<cv::Point>& matching);
      // checks whether point(x, y) is around point p
      bool around_point(int x, int y, cv::Point& p);
      // checks whether point(x, y) is around any point p in the list, returns such p if found
      bool around_points(int x, int y, std::list<cv::Point>& matching, cv::Point** p_ptr_ptr);
};

}  // namespace custom_landmark_2d