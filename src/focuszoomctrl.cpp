#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
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

int focus, zoom = 170, zoommax = 170;
int focmin = 400;
int focmax = 800;
int focrange = 100;
int focrangecenter;
int lfoc, hfoc;
int focusstep = 2;
int delay;
int smalldelay = 7;
int largedelay = 2*delay;
bool detectflag = false, scanfocus = true, direction = FAR,firstscan = true;
int scanfocmax;
int focuscounter = 0;
int zoomcounter = 0;
int notdetectcounter = 0;
double sharpness, sharpnessold = 0;


void sendFocus(){
	ostringstream cmd_buff;
	cmd_buff << ">f,"<<focus<<"<";
	string cmd = cmd_buff.str();
	write(serial_fd,cmd.c_str(),cmd.length());
//	ROS_INFO("Sent focus: [%i], Sharpness [%f]", focus,sharpness);
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

void scanFocus(){

	if(firstscan){
		delay = largedelay;
		focus = lfoc;
		sendFocus();
		firstscan = false;
	}
	if(focuscounter>delay){
		delay = smalldelay;
		focuscounter = 0;
		if(sharpness>sharpnessold){
			scanfocmax = focus;
		}
		focus += 50;
		if(focus>hfoc){
			scanfocus = false;
			focus = scanfocmax;
			firstscan  = true;
		}
		else{
			sendFocus();
			}
		sharpnessold = sharpness;
	}
}

void controlFocus(){
	if(focuscounter>smalldelay){
	focuscounter = 0;
		if(sharpness<sharpnessold){
			direction ^= direction;
		}
		if(direction == NEAR) focus -= focusstep;
		else focus += focusstep;
		focus = min(hfoc, focus);
		focus = max(lfoc, focus);
		sendFocus();
	}
}

void detectCallback(const std_msgs::Bool::ConstPtr& detectmsg){
	detectflag = detectmsg->data;
}

void focusCallback(const std_msgs::Float64::ConstPtr& focusmsg){
	sharpness = focusmsg->data;
	focuscounter++;
	if(!detectflag){
		notdetectcounter++;
		scanfocus = true;

		if(notdetectcounter > 5){
			lfoc = focmin;
			hfoc = focmax;
			notdetectcounter = 0;
		}
		else{
			lfoc =  (1-(double)zoom/(double)zoommax)*300+400;
			hfoc =  (1-(double)zoom/(double)zoommax)*300+500;
		}
	}
	if (scanfocus){
		 scanFocus();
	}
	else controlFocus();
	
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
	ros::Subscriber detectsub = n.subscribe("detectmsg",1,detectCallback);
	ros::spin();

	return(0);
}

