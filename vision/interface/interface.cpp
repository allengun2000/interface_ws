#define PI 3.14159265
#include "interface.hpp"
#include "math.h"
#define FRAME_COLS 640
#define FRAME_ROWS 480
#define IMAGE_TEST1 "/home/interface_ws/src/interface_ws/vision/1.jpg"//圖片路徑
static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
const double ALPHA = 0.5;


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
    //std::cout<<BallHSVBoxMsg[3]<<std::endl;

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
	ros::NodeHandle nh("~");

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
    frame=new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
    ColorModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
} 
void InterfaceProc::Parameter_setting(const int x){
////////////////////////////////HSV設定///////////////////////////////////////////
	HSV_init[0] = 0; HSV_init[1] = 360;
    HSV_init[2] = 0; HSV_init[3] = 255;
    HSV_init[4] = 0; HSV_init[5] = 255;

	nh.setParam("Colormode",ColorModeMsg);
	if(nh.hasParam("Colormode")){
		for(int i=0;i<6;i++){

			HSV_red.push_back(BallHSVBoxMsg[i]);  HSV_green.push_back(BlueHSVBoxMsg[i]);
            HSV_blue.push_back(GreenHSVBoxMsg[i]); HSV_yellow.push_back(YellowHSVBoxMsg[i]);
			}
		}
		else{
			for(int i=0;i<6;i++){

			HSV_red.push_back(HSV_init[i]);  HSV_green.push_back(HSV_init[i]);
      	  	HSV_blue.push_back(HSV_init[i]); HSV_yellow.push_back(HSV_init[i]);
			}
		}
	nh.setParam("/HSV/Ball",HSV_red);
	nh.setParam("/HSV/Blue",HSV_blue);
	nh.setParam("/HSV/Yellow",HSV_yellow);
	nh.setParam("/HSV/Green",HSV_green);
/////////////////////////////////掃瞄點前置參數///////////////////////////////////
	scan_para.push_back(Angle_Near_GapMsg);
	scan_para.push_back(Magn_Near_GapMsg);
	scan_para.push_back(Magn_Near_StartMsg);
	scan_para.push_back(Magn_Middle_StartMsg);
	scan_para.push_back(Magn_Far_StartMsg);
	scan_para.push_back(Magn_Far_EndMsg);
	scan_para.push_back(Dont_Search_Angle_1Msg);
	scan_para.push_back(Dont_Search_Angle_2Msg);
	scan_para.push_back(Dont_Search_Angle_3Msg);
	scan_para.push_back(Angle_range_1Msg);
	scan_para.push_back(Angle_range_2_3Msg);
	nh.setParam("scan_para",scan_para);
///////////////////////////////////FPS////////////////////////////////////////////////
	nh.setParam("FPS",fpsMsg);
/////////////////////////////////////////////////////////////////////////////////////
}
InterfaceProc::~InterfaceProc()
{
	delete frame;
    delete ColorModels;
	cv::destroyWindow(OPENCV_WINDOW);
}
/////////////////////////////////影像讀進來//////////////////////////////////////////
void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	
    frame_counter++;
    static long int StartTime = ros::Time::now().toNSec();
    static double FrameRate = 0.0;

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	}catch (cv_bridge::Exception& e)	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	*frame = cv_ptr->image;
    *ColorModels =ColorModel(*frame);

	// Image Output
    cv::imshow(OPENCV_WINDOW, *ColorModels);
//	sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", *thresholdImg16).toImageMsg();
//	image_pub_threshold_.publish(thresholdMsg);

	cv::waitKey(3);
/////////////////////////FPS設定///////////////////////////////////////	
    if(frame_counter == 10){
      EndTime = ros::Time::now().toNSec();
	if(fpsMsg!=0){
      dt = (EndTime - StartTime)/frame_counter;
      StartTime = EndTime;
      if( dt!=0 )
      {
              FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
              //cout << "FPS: " << FrameRate << endl;
      }
	}
      frame_counter = 0;
   }
		fpsMsg=FrameRate;

	//camera.publish(camera.msg);
//////////////////////////////////////////////////////////////////////


}
cv::Mat InterfaceProc::ColorModel(const cv::Mat iframe)
{
	static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
    for (int i = 0; i < iframe.rows; i++) {
        for (int j = 0; j < iframe.cols; j++) {
            double B = iframe.data[(i*iframe.cols*3)+(j*3)+0]+0;
            double G = iframe.data[(i*iframe.cols*3)+(j*3)+1]+0;
            double R = iframe.data[(i*iframe.cols*3)+(j*3)+2]+0;
            double H,S,V;
            V =(max(R,G)>max(G,B))?max(R,G):max(G,B);   //max(R,G,B);
            double mn=(min(R,G)<min(G,B))?min(R,G):min(G,B);//min(R,G,B);
            if(R==V){H=(G-B)*60/(V-mn);}
            if(G==V){H=120+(B-R)*60/(V-mn);}
            if(B==V){H=240+(R-G)*60/(V-mn);}
            if(H<0){H=H+360;}
            S=(((V-mn)*100)/V);
            switch(ColorModeMsg){
            case 0:
                 hmax = BallHSVBoxMsg[1];
                 hmin = BallHSVBoxMsg[0];
                 smax = BallHSVBoxMsg[3];
                 smin = BallHSVBoxMsg[2];
                 vmax = BallHSVBoxMsg[5];
                 vmin= BallHSVBoxMsg[4];
                break;
            case 1:
                hmax = GreenHSVBoxMsg[1];
                hmin = GreenHSVBoxMsg[0];
                smax = GreenHSVBoxMsg[3];
                smin = GreenHSVBoxMsg[2];
                vmax = GreenHSVBoxMsg[5];
                vmin= GreenHSVBoxMsg[4];
                break;
            case 2:
                hmax = BlueHSVBoxMsg[1];
                hmin = BlueHSVBoxMsg[0];
                smax = BlueHSVBoxMsg[3];
                smin = BlueHSVBoxMsg[2];
                vmax = BlueHSVBoxMsg[5];
                vmin= BlueHSVBoxMsg[4];
                break;
            case 3:
                hmax = YellowHSVBoxMsg[1];
                hmin = YellowHSVBoxMsg[0];
                smax = YellowHSVBoxMsg[3];
                smin = YellowHSVBoxMsg[2];
                vmax = YellowHSVBoxMsg[5];
                vmin= YellowHSVBoxMsg[3];
                break;
            case 4:
                hmax = WhiteHSVBoxMsg[1];
                hmin = WhiteHSVBoxMsg[0];
                smax = WhiteHSVBoxMsg[3];
                smin =WhiteHSVBoxMsg[2];
                vmax = WhiteHSVBoxMsg[5];
                vmin= WhiteHSVBoxMsg[4];
                break;
            }

         if((H<=hmax)&&(S<=smax)&&(V<=vmax)&&(H>=hmin)&&(S>=smin )&&(V>=vmin) ){
            oframe.data[(i*iframe.cols*3)+(j*3)+0] = 0;
            oframe.data[(i*iframe.cols*3)+(j*3)+1] = 0;
            oframe.data[(i*iframe.cols*3)+(j*3)+2] = 0;}else{
             oframe.data[(i*iframe.cols*3)+(j*3)+0]=iframe.data[(i*iframe.cols*3)+(j*3)+0];
             oframe.data[(i*iframe.cols*3)+(j*3)+1]=iframe.data[(i*iframe.cols*3)+(j*3)+1];
             oframe.data[(i*iframe.cols*3)+(j*3)+2]=iframe.data[(i*iframe.cols*3)+(j*3)+2] ;
         }
		}

    }

	system("rosparam dump ~/interface_ws/Parameter.yaml");
	return oframe;
}
