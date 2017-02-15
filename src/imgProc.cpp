#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2//objdetect.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/tracking.hpp"
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
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
bool detectflag = false;
bool firstloop_flag = false;
bool PTUcontrol = false;
double Kp = 0;
double Ki = 0;
double Kd = 0;
double pRes = 46.2857 / 3600 * M_PI / 180; // rad per position
double tRes = 11.5714 / 3600 * M_PI / 180; // rad per position
double alpha1 = 0.079555;	// rad width angle for f  ca 80 mm, look-up table needed wrong previous alpha1 = 0.121884776
double alpha2 = 0.044814;	// rad width angle for f ca 80 , look up table needed. wrong previous alpha2 = 0.0687929976
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
dt = 0.033;	// time step = querry and send command
double u1;	// pan input
double u2;	// tilt input
double tside; 	// target rect side length
double zoomaverage =0;
double loopcounterzoom = 0;
double measuredPan = 0, measuredTilt = 0;
// write to file variables:
ofstream myfile;
bool writeData;
int filenumber = 0;
struct timeval t1, t2;
double elapsedTime, timebegin;
vector<double> timeVec;
vector<double> e1Vec;
vector<double> e2Vec;
vector<double> u1Vec;
vector<double> u2Vec;
vector<double> i1Vec;
vector<double> i2Vec;
vector<double> d1Vec;
vector<double> d2Vec;
vector<double> panposVec;
vector<double> tiltposVec;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Detector
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void DetectColor(Mat frame) {
	Mat  frameThr, frameHSV;
	cvtColor(frame, frameHSV, COLOR_BGR2HSV);
	inRange(frameHSV, Scalar(75, 40, 60), Scalar(105, 255, 255), frameThr);
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
		oe1 = e1;
		oe2 = e2;
		e1 = alpha1 * (double)(target.x - cWidth)/(double)cWidth;	//atan((target.x - cWidth)*tan(alpha1) / cWidth);
		e2 = alpha2 * (double)(target.y - cHeight)/(double)cHeight;	//atan((target.y - cHeight)*tan(alpha2) / cHeight);
		ie1 += e1*dt;
		ie2 += e2*dt;
		de1 = (e1 - oe1) / dt;
		de2 = (e2 - oe2) / dt;
		targetRect = Rect(target.x - tside / 2, target.y - tside / 2, tside, tside) & Rect(0,0,frWidth,frHeight);
		rectangle(frame, targetRect, Scalar(0, 0, 255), 2, 8, 0);
		if (firstloop_flag) {
			u1 = -(Kp*e1 + Ki*ie1 + Kd*de1) / pRes;
			u2 = -(Kp*e2 + Ki*ie2 + Kd*de2) / tRes;
		}
		else {
			firstloop_flag = true;
			u1 = -(Kp*e1) / pRes;
			u2 = -(Kp*e2) / tRes;
		}
//		cout <<targetRect.x << " " <<targetRect.y<< " "  << targetRect.width+targetRect.x<< " "  << targetRect.height+targetRect.y << "\n";
		detectflag = true;
	}
	else {
		e1 = 0;
		e2 = 0;
		de1 = 0;
		de2 = 0;
		ie1 = 0;
		ie2 = 0;
		oe1 = 0;
		oe2 = 0;
		u1 = 0 ;
		u2 = 0;
		firstloop_flag = false;
		detectflag = false;
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
		if (abs(e2) > 0.00244) { // 0.0174533 corresponds to 1deg, 0.03491, 0.00244 = 0.14deg
			u2 = copysign(240, u2);
		}
		else {
			u2 = 0;
		}
	}
	else if (abs(u2) > 10000) u2 = copysign(10000, u2);
	//u2 = 0, e2 = 0, ie2 = 0, de2 =0;
	if(PTUcontrol&&writeData){
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//cout << setprecision(14) <<elapsedTime << "\n";
		timeVec.push_back(elapsedTime);
		e1Vec.push_back(e1/pRes);
		e2Vec.push_back(e2/tRes);
		u1Vec.push_back(u1);
		u2Vec.push_back(u2);
		i1Vec.push_back(ie1*Ki/pRes);
		i2Vec.push_back(ie2*Ki/tRes);
		d1Vec.push_back(de1*Kd/pRes);
		d2Vec.push_back(de2*Kd/tRes);
		panposVec.push_back(measuredPan/pRes);
		tiltposVec.push_back(measuredTilt/tRes);
	}
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
// PTU Poistion Callback
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ptuPosCallback(const std_msgs::Float64MultiArray::ConstPtr& ptuposmsg){
	measuredPan = ptuposmsg->data[0] * pRes;
	measuredTilt = ptuposmsg->data[1] * tRes;
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
	ros::Publisher detect_pub = n.advertise<std_msgs::Bool>("detectmsg",1);
	ros::Publisher ptuhomemsg_pub = n.advertise<std_msgs::Bool>("ptuhomemsg",1);
	ros::Subscriber ptupos_sub = n.subscribe("ptuposmsg",1,ptuPosCallback);
	std_msgs::Bool ptuhomemsg;	
	std_msgs::Bool detectmsg;
	std_msgs::Float64 focusmsg;
	std_msgs::Float64MultiArray ptumsg;
	std_msgs::Int16 zoommsg;
	zoommsg.data = 170;
	ptumsg.data.resize(2);

	VideoCapture cap;
	cap.open(0);
	if(!cap.isOpened()){
		cout << "Cannot open camera!!!\n";
		return -1;
	}
	Mat frame, grayframe;
	string winName = "Preview";
	namedWindow(winName,WINDOW_KEEPRATIO);
	cap.set(CAP_PROP_FPS,fps);
	cap.set(CAP_PROP_FRAME_HEIGHT,frHeight);
	cap.set(CAP_PROP_FRAME_WIDTH,frWidth);
	long int frCounter = 0;


	int loopCounter = 0;

	while(true){	

		ros::spinOnce(); //  for callbacks

		cap>>frame;
		cvtColor(frame, grayframe, CV_BGR2GRAY);
		DetectColor(frame);
		PTUSpeedControl();
		if (!detectflag){
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
		if(c == 115){ //s: start ptu control
			PTUcontrol ^= true;
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0;
			firstloop_flag = false;
			gettimeofday(&t1, NULL);
		}
		if(c == 112){ // p: set new gain
			ptumsg.data[0] = 0;
			ptumsg.data[1] = 0;
			ptumsg_pub.publish(ptumsg);
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0;
			firstloop_flag = false;
			PTUcontrol = false;
			cout << "Please enter the desired gain (double, now: " << Kp << " ) \n";
			cin >> Kp;
			cout << "\n New PID is: " << Kp << " " <<  Ki<< " "<< Kd<<"\n";
		}
		if(c == 100){ // d: set new diff gain
			ptumsg.data[0] = 0;
			ptumsg.data[1] = 0;
			ptumsg_pub.publish(ptumsg);
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0;
			firstloop_flag = false;
			PTUcontrol = false;
			cout << "Please enter the desired differentiator gain (double, now: " << Kd << " )\n";

			cin >> Kd;
			cout << "\n New PID is: " << Kp << " " <<  Ki<< " "<< Kd<<"\n";
		}

		if(c == 105){ // p 
			ptumsg.data[0] = 0;
			ptumsg.data[1] = 0;
			ptumsg_pub.publish(ptumsg);
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0;
			firstloop_flag = false;
			PTUcontrol = false;
			cout << "Please enter the desired integrator gain (double): \n";
			cin >> Ki;
			cout << "\n New PID is: " << Kp << " " <<  Ki<< " "<< Kd<<"\n";
		}
		if (c == 116) {// t: open file
			writeData ^= true;
			if (writeData) {
				if (myfile.is_open()) myfile.close();
				stringstream ss;
				ss << "PanKp" << Kp << "Ki" << Ki << "Kd" << Kd << "_" <<setfill('0')<<setw(3)<< filenumber <<".txt";
				string filename = ss.str();
				myfile.open(filename.c_str());
				timeVec.clear();
				e1Vec.clear();
				e2Vec.clear();
				u1Vec.clear();
				u2Vec.clear();
				i1Vec.clear();
				i2Vec.clear();
				d1Vec.clear();
				d2Vec.clear();
				panposVec.clear();
				tiltposVec.clear();
				cout << "file " << ss.str() << " opened\n";
				myfile << "Kp	Ki	Kd \n";
				myfile << Kp << "," << Ki << "," << Kd << "\n";
				myfile << "time	e1	e2	u1	u2	i1	i2	d1	d2	pp	tp\n";
				filenumber++;
			}
			else {
				myfile.close();
				cout << "file closed\n";
			}
		}
		if (c == 119) {// w: write data to file
				for (int i = 0; i < timeVec.size(); i++) {
					myfile << setprecision(14) << timeVec[i] << "," <<
						//setprecision(8) << dtimeVec[i] << "," <<
						setprecision(8) << e1Vec[i] << "," <<
						setprecision(8) << e2Vec[i] << "," <<
						setprecision(8) << u1Vec[i] << "," <<
						setprecision(8) << u2Vec[i] << "," <<
						setprecision(8) << i1Vec[i] << "," <<
						setprecision(8) << i2Vec[i] << "," <<
						setprecision(8) << d1Vec[i] << "," <<
						setprecision(8) << d2Vec[i] << "," <<
						setprecision(8) << panposVec[i] << "," <<
						setprecision(8) << tiltposVec[i] << "\n";
				}
				cout << "Data written to file\n";
		}
		if(c == 114){ // r: rehome ptu
			ptumsg.data[0] = 0;
			ptumsg.data[1] = 0;
			ptumsg_pub.publish(ptumsg);
			e1 = 0; e2 = 0; de1 = 0; de2 = 0, ie1=0,ie2=0, oe1 = 0, oe2=0;
			firstloop_flag = false;
			PTUcontrol = false;
			ptuhomemsg.data = true;
			ptuhomemsg_pub.publish(ptuhomemsg);
		}
		// set focus msg
		focusmsg.data = focusmeasure;
		// set ptu msg
		if(PTUcontrol){
		ptumsg.data[0] = u1;
		ptumsg.data[1] = u2;
		//loopCounter++;
		//cout<< loopCounter<< "\n";
		}
		else{
		ptumsg.data[0] = 0;
		ptumsg.data[1] = 0;
		}
		// set zoom msg
		if(detectflag){
			loopcounterzoom++;
			if(loopcounterzoom > 119)zoomaverage +=tside;
			if(loopcounterzoom > 149){
				zoomaverage /= loopcounterzoom-119;
				if(zoomaverage > 210){
					zoommsg.data+=3;
					zoommsg.data-=3;
				}
				if (zoomaverage<200) {
					zoommsg.data-=3;
				}
				loopcounterzoom = 0;
				zoomaverage=0,
				zoommsg.data = min((int)zoommsg.data,(int)170);
				zoommsg.data = max((int)zoommsg.data,(int)10);
				zoom_pub.publish(zoommsg);
			}
		}
		else {
			loopcounterzoom = 0;
			zoommsg.data = 170;
		}
		// set detect msg
		detectmsg.data = detectflag;



		// publish
		ptumsg_pub.publish(ptumsg);
		focus_pub.publish(focusmsg);
		detect_pub.publish(detectmsg);

	}
	return 0;
}
