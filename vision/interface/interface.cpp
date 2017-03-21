#include "interface.hpp"
#include "math.h"
#define PI 3.14159265
#define FRAME_COLS 640
#define FRAME_ROWS 480
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;

void InterfaceProc::ParameterButtonCall (const vision::parameterbutton msg)
{
    buttonmsg=msg.button;
    std::cout<<buttonmsg<<std::endl;
}
void InterfaceProc::colorcall(const vision::color msg){
     ColorModeMsg=msg.ColorMode;
     switch(ColorModeMsg){
     case 0:
     for(int i=0;i<6;i++)
         BallHSVBoxMsg[i]=msg.BallHSVBox[i];
         break;
     case 1:
     for(int i=0;i<6;i++)
         GreenHSVBoxMsg[i]=msg.GreenHSVBox[i];
         break;
     case 2:
     for(int i=0;i<6;i++)
         BlueHSVBoxMsg[i]=msg.BlueHSVBox[i];
         break;
     case 3:
     for(int i=0;i<6;i++)
         YellowHSVBoxMsg[i]=msg.YellowHSVBox[i];
         break;
     case 4:
     for(int i=0;i<6;i++)
         WhiteHSVBoxMsg[i]=msg.WhiteHSVBox[i];
         break;
     }
    std::cout<<BallHSVBoxMsg[3]<<std::endl;

}
void InterfaceProc::centercall(const vision::center msg){
        CenterXMsg=msg.CenterX;
        CenterYMsg=msg.CenterY;
        InnerMsg=msg.Inner;
        OuterMsg=msg.Outer;
        FrontMsg=msg.Front;
        Camera_HighMsg=msg.Camera_High;
}
void InterfaceProc::whitecall(const vision::white msg){
    WhiteGrayMsg=msg.Gray;
    WhiteAngleMsg=msg.Angle;
}
void InterfaceProc::cameracall(const vision::camera msg){
fpsMsg=msg.fps;
}
void InterfaceProc::blackcall(const vision::black msg){
    BlackGrayMsg=msg.Gray;
    BlackAngleMsg=msg.Angle;
}
void InterfaceProc::colorbuttoncall(const vision::colorbutton msg){
 colorbottonMsg=msg.button;
}
void InterfaceProc::scancall(const vision::scan msg){
Angle_Near_GapMsg=msg.Angle_Near_Gap;
Magn_Near_GapMsg=msg.Magn_Near_Gap;
Magn_Near_StartMsg=msg.Magn_Near_Start;
Magn_Middle_StartMsg=msg.Magn_Middle_Start;
Magn_Far_StartMsg=msg.Magn_Far_Start;
Magn_Far_EndMsg=msg.Magn_Far_End;
Dont_Search_Angle_1Msg=msg.Dont_Search_Angle_1;
Dont_Search_Angle_2Msg=msg.Dont_Search_Angle_2;
Dont_Search_Angle_3Msg=msg.Dont_Search_Angle_3;
Angle_range_1Msg=msg.Angle_range_1;
Angle_range_2_3Msg=msg.Angle_range_2_3;
}
InterfaceProc::InterfaceProc()
	:it_(nh)
{
	ros::NodeHandle n("~");

	image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
	image_pub_threshold_ = it_.advertise("/interface/image_raw/threshold", 1);
	s1 = nh.subscribe("/interface/parameterbutton", 1000, &InterfaceProc::ParameterButtonCall, this);
    s2 = nh.subscribe("interface/color", 1000, &InterfaceProc::colorcall,this);
    s3 = nh.subscribe("interface/center", 1000, &InterfaceProc::centercall,this);
    s4 = nh.subscribe("interface/white", 1000, &InterfaceProc::whitecall,this);
    s5 = nh.subscribe("interface/camera", 1000, &InterfaceProc::cameracall,this);
    s6 = nh.subscribe("interface/black", 1000, &InterfaceProc::blackcall,this);
    s7 = nh.subscribe("interface/colorbutton", 1000, &InterfaceProc::colorbuttoncall,this);
    s8 = nh.subscribe("interface/scan", 1000, &InterfaceProc::scancall,this);
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
