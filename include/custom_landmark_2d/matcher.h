#include <opencv2/opencv.hpp>

namespace custom_landmark_2d {

class Matcher {

	public:
      int count_times; 		   // max #times allowed for resizing
      float raw_match_limit; // the threshold for BEST acceptable matching points
      float match_limit;     // the threshold for ALL acceptable matching points
      Matcher();
      // returns a vector of cv::Point that defines the upper-left corner of the 2-d bounding box
      // of matched templates in the scene, together with the width & height of the box.
      bool scale_match(const cv::Mat& scene, const cv::Mat& templ, 
      	               std::list<cv::Point>& lst,
      				         int* width, int* height);

 	private:
      int match_method;
      int x_dist, y_dist;
      // performs a single match on the given scene & templ, returns the max match_score	
      double match(const cv::Mat& scene, const cv::Mat& templ, std::list<cv::Point>& matching);
      // checks whether point(x, y) is around point p
      bool around_point(int x, int y, cv::Point& p);
      // checks whether point(x, y) is around any point p in the list, returns such p if found
      bool around_points(int x, int y, std::list<cv::Point>& matching, cv::Point** p_ptr_ptr);
};

}  // namespace custom_landmark_2d