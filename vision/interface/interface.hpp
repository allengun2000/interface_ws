#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>

class InterfaceProc
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_threshold_;

	cv::Mat *frame;
	cv::Mat *gray;

public:
  InterfaceProc();
  ~InterfaceProc();

	void imageCb(const sensor_msgs::ImageConstPtr&);
	cv::Mat Gray(const cv::Mat iframe);
};
