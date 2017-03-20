#include "interface.hpp"
#include "math.h"
#define PI 3.14159265
#define FRAME_COLS 640
#define FRAME_ROWS 480
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;

void InterfaceProc::ParameterButtonCall (const vision::parameterbutton msg)
{
	std::cout<<msg.button<<std::endl;
}

InterfaceProc::InterfaceProc()
	:it_(nh)
{
	ros::NodeHandle n("~");

	image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
	image_pub_threshold_ = it_.advertise("/interface/image_raw/threshold", 1);
	s1 = nh.subscribe("/interface/parameterbutton", 1000, &InterfaceProc::ParameterButtonCall, this);

	cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
	frame = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
	gray = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
} 

InterfaceProc::~InterfaceProc()
{
	delete frame;
	delete gray;
	cv::destroyWindow(OPENCV_WINDOW);
}

void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	}catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	*frame = cv_ptr->image;
	*gray = Gray(*frame);

	// Image Output
	cv::imshow(OPENCV_WINDOW, *gray);
//	sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", *thresholdImg16).toImageMsg();
//	image_pub_threshold_.publish(thresholdMsg);

  cv::waitKey(3);
}
cv::Mat InterfaceProc::Gray(const cv::Mat iframe)
{
	static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
	int gray;
	for (int h = 0; h < iframe.rows; h++) {
		for (int w = 0; w < iframe.cols; w++) {
			gray = (iframe.data[(h*iframe.cols*3)+(w*3)+0] +
							iframe.data[(h*iframe.cols*3)+(w*3)+1] +
							iframe.data[(h*iframe.cols*3)+(w*3)+2])/3;
			oframe.data[(h*iframe.cols*3)+(w*3)+0] = gray;
			oframe.data[(h*iframe.cols*3)+(w*3)+1] = gray;
			oframe.data[(h*iframe.cols*3)+(w*3)+2] = gray;
		}
	}
	return oframe;
}
