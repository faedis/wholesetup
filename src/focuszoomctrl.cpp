#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
//#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2//objdetect.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/tracking.hpp"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <cstring>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

#define BAUDRATE B9600
#define SERIAL_PATH "/dev/ttyACM1"
//Setup Serial communication
struct termios serial_settings;
// Try opening serial port
// int serial_fd = open(SERIAL_PATH,O_WRONLY|O_NOCTTY);//serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY);
int serial_fd;


#define NEAR 0
#define FAR 1
#define OFF 0
#define ON 1

int framecounter = 0;
int zoomframecounter = 0;
int focus, focusMax, zoom;
int focusBasestep=30;
int focusStep = focusBasestep;
bool direction, dirChanged; // (NEAR or FAR)
double sharpness, sharpnessOld;
double sharpnessScanMax;
bool scanfocus = ON;
bool controlfocus = OFF;
bool movefocus = OFF;
bool largeDelay = ON;
int delayThreshold;
int focusScanMax;
int lTh = 0; // lower hystheresis threshold
int hTh = 30; // higher hystheresis trheshold

void sendFocus(){
	ostringstream cmd_buff;
	cmd_buff << ">f,"<<focus<<"<";
	string cmd = cmd_buff.str();
	write(serial_fd,cmd.c_str(),cmd.length());
	ROS_INFO("Sent focus: [%i], Sharpness [%f]", focus,sharpness);
	//cout<<"Sent focus command: "<<cmd<<endl;
}
void sendZoom(){
	ostringstream cmd_buff;
	cmd_buff << ">z,"<<zoom<<"<";
	string cmd = cmd_buff.str();
	write(serial_fd,cmd.c_str(),cmd.length());
	//ROS_INFO("Sent zoom: [%i]", zoom);
	//cout<<"Sent focus command: "<<cmd<<endl;
}
void focusScan() {

	if (largeDelay) {
			sharpnessScanMax = 0;
			focusScanMax = 0;
			focus = 300;
			sendFocus();
			delayThreshold = 35;
			largeDelay = OFF;
		}
	if(!largeDelay & (framecounter>delayThreshold)){
		framecounter = 0;
		delayThreshold = 7;

		if (sharpness > sharpnessScanMax) {
		// this must be improved at some point, but might be enough for lab tests
			sharpnessScanMax = sharpness;
			focusScanMax = focus;
		//	cout << "focus: " << focus << "sharpness: " << sharpness << "\n";
		}
		if (focus < 800) {
			focus += 100;
			sendFocus();
		}
		else {
			focus = focusScanMax - 50;
			sharpnessOld = sharpnessScanMax;
			sendFocus();
			cout << focus << "\n";
			scanfocus = OFF;
			largeDelay = ON;
			controlfocus = ON;
		}
	}
}
void controlFocus() {
	if (framecounter > 7) {
		framecounter = 0;
		if (sharpness < sharpnessOld) {
			direction = !direction;
			focusStep *= 0.8;
			focusStep = max(focusStep, 2);
			dirChanged = true;
		}
		movefocus = ON;
	
		if (abs(sharpness - sharpnessOld) > 0.05) {
			if (sharpness < sharpnessOld) {
				direction = !direction;
				if (dirChanged == true) {
					focusStep = ceil((double)focusStep / 2.0);
				}
				dirChanged = true;
			}
			else {
				dirChanged = false;
				focusStep *= 2;
				if (focusStep > focusBasestep) focusStep = focusBasestep;
			}
		movefocus = ON;
		}
	sharpnessOld = sharpness;
	}
}
void moveFocus() {	
	if (direction == FAR) {
		focus = focus + focusStep;
	}
	else if (direction == NEAR) {
		focus = focus - focusStep;
	}
	if (focus < 300)focus = 300;
	else if (focus > 800)focus = 800;
	if (sharpness - sharpnessScanMax < -1) {
		focusStep = focusBasestep;
	}
	sendFocus();	
	movefocus = OFF;
}
void focusCallback(const std_msgs::Float64::ConstPtr& focusmsg){
	sharpness = focusmsg->data;
	framecounter++;
	if(scanfocus)focusScan();
	else if(controlfocus) controlFocus();
	if(movefocus) moveFocus();

}
void zoomCallback(const std_msgs::Int16::ConstPtr& zoommsg){
	zoom = zoommsg->data;
	sendZoom();
}
int main( int argc, char** argv ) {
	// Open serial port to arduino++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY|O_NDELAY);
	if(serial_fd == -1){
		printf("Serial Port connection Failed.\n");
		return -1;
	}
	else{
		//Get serial port settings
		tcgetattr(serial_fd, &serial_settings); //Get Current Settings of the Port
		cfsetispeed(&serial_settings,BAUDRATE); //Set Input Baudrate
		cfsetospeed(&serial_settings,BAUDRATE); //Set Output Baudrate
		serial_settings.c_cflag &= ~PARENB; //Mask Parity Bit as No Parity
		serial_settings.c_cflag &= ~CSTOPB; //Set Stop Bits as 1 or else it will be 2
		serial_settings.c_cflag &= ~CSIZE; //Clear the current no. of data bit setting
		serial_settings.c_cflag |= CS8; //Set no. of data bits as 8 Bits

		serial_settings.c_iflag = 0; //Mask Parity Bit as No Parity
		serial_settings.c_oflag = 0;
		serial_settings.c_lflag = 0;
	if(tcsetattr(serial_fd, TCSAFLUSH, &serial_settings) < 0){
		printf("Setting up Serial Port settings Failed");
		return -1;
	}
	else
		printf("Serial Port connection Success.\n\n");
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	ros::init(argc,argv,"focuszoomctrl_node");

	ros::NodeHandle n;
	
	ros::Subscriber focussub = n.subscribe("focusmsg",1,focusCallback);
	ros::Subscriber zoomsub = n.subscribe("zoommsg",1,zoomCallback);

	ros::spin();

	return(0);
}

