#include <cstdio>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <vision/parameterbutton.h>
#include "vision/parametercheck.h"
#include "vision/center.h"
#include "vision/white.h"
#include "vision/camera.h"
#include "vision/black.h"
#include "vision/colorbutton.h"
#include "vision/scan.h"
#include "vision/color.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
using namespace cv;


class InterfaceProc
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_threshold_;
  ros::Subscriber s1;
  ros::Subscriber s2;
  ros::Subscriber s3;
  ros::Subscriber s4;
  ros::Subscriber s5;
  ros::Subscriber s6;
  ros::Subscriber s7;
  ros::Subscriber s8;
	ros::Subscriber s9;



	cv::Mat *frame;
	cv::Mat *centermats;

 	



  cv::Mat *CenterModels;
  cv::Mat *ColorModels;
  cv::Mat *CameraModels;
  cv::Mat *ScanModels;
  cv::Mat *outputframe;
 int hmax,hmin,smax,smin,vmax,vmin;

public:
  InterfaceProc();
  ~InterfaceProc();
  int buttonmsg;
  int CenterXMsg;
  int CenterYMsg;
  int robotCenterX;
  int robotCenterY;
  int InnerMsg;
  int OuterMsg;
  int FrontMsg;
  int camera_focal;
  int Camera_HighMsg;
  int colorbottonMsg;
  int Angle_Near_GapMsg;
  int Magn_Near_GapMsg;
  int Magn_Near_StartMsg;
  int Magn_Middle_StartMsg;
  int Magn_Far_StartMsg;
  int Magn_Far_EndMsg;
  int Dont_Search_Angle_1Msg;
  int Dont_Search_Angle_2Msg;
  int Dont_Search_Angle_3Msg;
  int Angle_range_1Msg;
  int Angle_range_2_3Msg;
  int BlackGrayMsg;
  int BlackAngleMsg;
  int WhiteGrayMsg;
  int WhiteAngleMsg;
  int fpsMsg;
  int BallHSVBoxMsg[6];
  int GreenHSVBoxMsg[6];
  int BlueHSVBoxMsg[6];
  int YellowHSVBoxMsg[6];
  int WhiteHSVBoxMsg[6];
  int ColorModeMsg;
  int paraMeterCheck;

	void imageCb(const sensor_msgs::ImageConstPtr&);
	void ParameterButtonCall(const vision::parameterbutton);
  void colorcall(const vision::color);
  void centercall(const vision::center);
  void whitecall(const vision::white);
  void cameracall(const vision::camera);
  void blackcall(const vision::black);
  void colorbuttoncall(const vision::colorbutton);
  void scancall(const vision::scan);
	void Parameter_getting(const int x) ;
	void Parameter_setting(const vision::parametercheck) ;

	int mosue_x,mosue_y;
  int distance_space[100];
  int distance_pixel[100];
	
///////////////////////FPS////////////////////////
  int frame_counter;
  long int EndTime;
  long int dt;
  void set_campara(int value_ex){
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;
    double exposure = (double)value_ex/1000000;
    double_param.name = "exposure";
    double_param.value = exposure;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;
    ros::service::call("/prosilica_driver/set_parameters", srv_req, srv_resp);
    }

    double camera_exposure;
    void get_campara(){
      camera_exposure = 0;
      nh.getParam("/prosilica_driver/exposure",camera_exposure);
	
    }

//////////////////////SCAN/////////////////////
  std::vector<int>scan_para;
  std::vector<int>scan_near;
  std::vector<int>scan_middle;
  std::vector<int>scan_far;

  std::vector<int>dis_space;
  std::vector<int>dis_pixel;
/////////////////////HSV///////////////////////
	int HSV_init[6];
  vector<int> HSV_red;
  vector<int> HSV_green;
  vector<int> HSV_blue;
  vector<int> HSV_yellow;



  double camera_f(int Omni_pixel);
  double Omni_distance(int object_x , int object_y);
  int Angle_Interval(int);

  cv::Mat ColorModel(const cv::Mat iframe);
  cv::Mat CenterModel(const cv::Mat iframe);
  cv::Mat ScanModel(const cv::Mat iframe);
  cv::Mat CameraModel(const cv::Mat iframe);

};
