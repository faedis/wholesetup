#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2//objdetect.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/tracking.hpp"
#include <sstream>
#include <iostream>
#include <sys/time.h>



using namespace std;
using namespace cv;

// Global Variables
Point target;
int frHeight = 360;
int frWidth = 640;
int cWidth = frWidth/2;
int cHeight = frHeight/2;
int fps = 30;
float focusmeasure = 0;
Rect targetRect;
bool detect_flag = false;
bool firstloop_flag = false;
bool PTUcontrol = false;
double Kp = 0;
double Ki = 0;
double Kd = 0;
double pRes = 46.2857 / 3600 * M_PI / 180; // rad per position
double tRes = 11.5714 / 3600 * M_PI / 180; // rad per position
double alpha1 = 0.121884776;	// rad width angle for f = 75mm, look-up table needed
double alpha2 = 0.0687929976;	// rad width angle for f = 75mm
double e1,	// error phi target-camera
e2,		// error theta target-camera
ie1 = 0,	// integral error phi
ie2 = 0,	// integral error theta
oe1 = 0,
oe2 = 0,
de1,		// derivative of error phi
de2,		// derivative of error theta
t = 0,		// current time
ot = 0,		// previous time
dt = 0.032;	// time step = querry and send command
double u1;	// pan input
double u2;	// tilt input
double tside; 	// target rect side length
double zoomaverage =0;
double loopcounterzoom = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Detector
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void DetectColor(Mat frame) {
	Mat  frameThr, frameHSV;
	cvtColor(frame, frameHSV, COLOR_BGR2HSV);
	inRange(frameHSV, Scalar(80, 40, 60), Scalar(115, 255, 255), frameThr);
	// morph opening
	erode(frameThr, frameThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(frameThr, frameThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	//morph closing
	dilate(frameThr, frameThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(frameThr, frameThr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	/*imshow("thr", frameThr);*/
	Moments frameMom = moments(frameThr/255); // m01/m00 = posY, m10/m00 = posX
	//cout << frameHSV.at<uchar>
	if (frameMom.m00 > 300) {
		double tArea = frameMom.m00;
		tside = sqrt(tArea);
		target = Point(frameMom.m10 / tArea, frameMom.m01 / tArea);
		e1 = atan((target.x - cWidth)*tan(alpha1) / cWidth);
		e2 = atan((target.y - cHeight)*tan(alpha2) / cHeight);
		ie1 += e1*dt;
		ie2 += e2*dt;
		de1 = (e1 - oe1) / dt;
		de2 = (e2 - oe2) / dt;
		targetRect = Rect(target.x - tside / 2, target.y - tside / 2, tside, tside) & Rect(0,0,frWidth,frHeight);
		rectangle(frame, targetRect, Scalar(0, 0, 255), 2, 8, 0);
		if (firstloop_flag) {
			u1 = -(Kp*e1 + Ki*ie1 + Kd*de1) / pRes;
			u2 = -(Kp*e2 + Kd*de2) / tRes;
		}
		else {
			firstloop_flag = true;
			u1 = -(Kp*e1 + Ki*ie1) / pRes;
			u2 = -(Kp*e2 + Ki*ie2)/ tRes;
		}
//		cout <<targetRect.x << " " <<targetRect.y<< " "  << targetRect.width+targetRect.x<< " "  << targetRect.height+targetRect.y << "\n";
		detect_flag = true;
	}
	else {
		e1 = 0;
		e2 = 0;
		de1 = 0;
		de2 = 0;
		ie1 = 0;
		ie2 = 0;
		u1 = 0 ;
		u2 = 0;
		firstloop_flag = false;
		detect_flag = false;
		target = Point(cWidth, cHeight);
	}


}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PTU control
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PTUSpeedControl() {
	//u1
	if (abs(u1) < 58) {
		if (abs(e1) > 0.00244) {
			u1 = copysign(60, u1);
		}
		else {
			u1 = 0;
		}
	}
	else if (abs(u1) > 10000) u1 = copysign(10000, u1);
	//u2	
	if (abs(u2) < 240) {
		if (abs(u1) > 0.00244) { // 0.0174533 corresponds to 1deg, 0.03491, 0.00244 = 0.14deg
			u2 = copysign(240, u2);
		}
		else {
			u2 = 0;
		}
	}
	else if (abs(u2) > 10000) u2 = copysign(10000, u2);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// focus measure
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float LaplaceVarFocus(Mat inputFrame) {
	if (inputFrame.empty()) {
		cout << "Invalid input for Focus calculation" << endl;
		return -1;
	}

	int ddepth = CV_32F;
	int kernel_size = 1;
	int scale = 1;
	int delta = 0;
	Mat lap;
	Mat viz;
	// GaussianBlur(inputFrame,inputFrame, Size(3,3), 0, 0, BORDER_DEFAULT );
	Laplacian(inputFrame, lap, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(lap, viz);
	Scalar mu, sigma;
	meanStdDev(lap, mu, sigma);
	float focusMeasure = sigma.val[0] * sigma.val[0];
	return focusMeasure;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// main()
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv){
	ros::init(argc, argv, "imgproc_node");
	
	ros::NodeHandle n;

	ros::Publisher ptumsg_pub = n.advertise<std_msgs::Float64MultiArray>("ptumsg",1); 
	ros::Publisher focus_pub = n.advertise<std_msgs::Float64>("focusmsg",1); 
	ros::Publisher zoom_pub = n.advertise<std_msgs::Int16>("zoommsg",1);

	std_msgs::Float64 focusmsg;
	std_msgs::Float64MultiArray ptumsg;
	std_msgs::Int16 zoommsg;
	zoommsg.data = 170;
	ptumsg.data.resize(2);
//ros::Rate loop_rate(10);

	VideoCapture cap;
	cap.open(0);
	if(!cap.isOpened()){
		cout << "Cannot open camera!!!\n";
		return -1;
	}
	Mat frame, grayframe;
	string winName = "Preview";
	namedWindow(winName,WINDOW_AUTOSIZE);
	cap.set(CAP_PROP_FPS,fps);
	cap.set(CAP_PROP_FRAME_HEIGHT,frHeight);
	cap.set(CAP_PROP_FRAME_WIDTH,frWidth);
	long int frCounter = 0;

	struct timeval t1, t2;
    	double elapsedTime;
	int loopCounter = 0;
	gettimeofday(&t1, NULL);
	while(true){	

		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    		elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
    		//cout << elapsedTime << " ms.\n";
		gettimeofday(&t1, NULL);
	
		cap>>frame;
		cvtColor(frame, grayframe, CV_BGR2GRAY);
		DetectColor(frame);
		PTUSpeedControl();
		if (!detect_flag){
			focusmeasure = LaplaceVarFocus(grayframe);
		}
		else{
//		ROS_INFO("Rectangle: [%i,%i,%i,%i]", targetRect.x, targetRect.y, targetRect.width+targetRect.x, targetRect.height+targetRect.y);
//		cout <<targetRect.x << " " <<targetRect.y<< " "  << targetRect.width+targetRect.x<< " "  << targetRect.height+targetRect.y << "\n";
			focusmeasure = LaplaceVarFocus(grayframe(targetRect));
		}
		string PTUStatus;
		if (PTUcontrol) {
			PTUStatus = "PTU control ON";
		}
		else {

			PTUStatus = "PTU control OFF";
		}
		putText(frame, PTUStatus, Point(50, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 0, 0), 1, 8);
		imshow(winName,frame);
		int c = waitKey(1);
		if(c == 27) break;
		if(c == 115){
			PTUcontrol ^= true;
		}
		if(c == 112){
			ptumsg.data[0] = 0;
			ptumsg.data[1] = 0;
			ptumsg_pub.publish(ptumsg);
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0;
			firstloop_flag = false;
			PTUcontrol = false;
			cout << "Please enter the desired gain (double, now: " << Kp << " ): \n";
			cin >> Kp;
			cout << "\n New gain is: " << Kp << "\n";
		}
		// set focus msg
		focusmsg.data = focusmeasure;
		// set ptu msg
		if(PTUcontrol){
		ptumsg.data[0] = u1;
		ptumsg.data[1] = u2;
		loopCounter++;
		cout<< loopCounter<< "\n";
		}
		else{
		ptumsg.data[0] = 0;
		ptumsg.data[1] = 0;
		}
		// set zoom msg
		if(detect_flag){
			loopcounterzoom++;
			if(loopcounterzoom > 119)zoomaverage +=tside;
			if(loopcounterzoom > 149){
				zoomaverage /= loopcounterzoom;
				if(zoomaverage > 110) zoommsg.data+=3;
				else if (zoomaverage<90) zoommsg.data-=3;
				zoomaverage=0, loopcounterzoom = 0;
				zoommsg.data = min((int)zoommsg.data,(int)170);
				zoommsg.data = max((int)zoommsg.data,(int)10);
				zoom_pub.publish(zoommsg);
			}
		
		}


		// publish
		ptumsg_pub.publish(ptumsg);
		focus_pub.publish(focusmsg);

	}
	return 0;
}
