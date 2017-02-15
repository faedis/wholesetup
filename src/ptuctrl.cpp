#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
extern "C" {
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "cpiver.h"
#include "ptu.h"
}


using namespace std;

struct timeval t1, t2;
double elapsedTime;
signed short int panSpeed, tiltSpeed, val, panZero = 0, tiltZero=0;
signed short int maxPos = 0;
signed short pPos = 0;
signed short tPos = 0;
signed short* pPtr = &pPos;
signed short* tPtr = &tPos;

bool firstloop_flag = false;
bool firstUse = false;
int loopCounter = 0;
	
	

class subAndpub{
public:
	subAndpub(){
		pub_ = n_.advertise<std_msgs::Float64MultiArray>("ptuposmsg",1);
		
		sub_ctrl = n_.subscribe("ptumsg",1,&subAndpub::ptuCallback,this);
		sub_home = n_.subscribe("ptuhomemsg",1,&subAndpub::ptuHomeCallback,this);
	}

	void ptuCallback(const std_msgs::Float64MultiArray::ConstPtr& ptumsg){
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
		//cout << elapsedTime << " ms.\n";
		//gettimeofday(&t1, NULL);
		//gettimeofday(&t1, NULL);
		panSpeed = floor(ptumsg->data[0]);
		tiltSpeed = floor(ptumsg->data[1]);
		ptu_set_desired_velocities(panSpeed, tiltSpeed);
		get_current_positions(pPtr,tPtr);
		std_msgs::Float64MultiArray ptuposmsg;
		ptuposmsg.data.resize(2);
		ptuposmsg.data[0] = *pPtr;
		ptuposmsg.data[1] = *tPtr;
		pub_.publish(ptuposmsg);
	}

	void ptuHomeCallback(const std_msgs::Bool::ConstPtr& ptuhomemsg){
		val = 1000;
		ptu_set_desired_velocities(val, val);
		set_desired_abs_positions(&panZero,&tiltZero);
		await_completion();
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_ctrl;
	ros::Subscriber sub_home;
};


//







int main( int argc, char** argv ) {
	// Set up PTU +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int filenumber = 0;
	char COMportName[256] = "/dev/ttyUSB0";
	int BaudRate = 9600;
	// Initialize PTU and set mode
	portstream_fd COMstream;			// creat PTU handle
	set_baud_rate(BaudRate);			// set baud rate
	COMstream = open_host_port(COMportName);	// open serial communication
	if (COMstream == PORT_NOT_OPENED)		// check connection is open, else abord
	{
		printf("\nSerial Port setup error.\n");
		return -1;
	}
	else{
		cout << "Serial port to PTU opened\n\n";
	}
	cout << "Do you want to reset PTU (suggested when powered up again)\nThen press 1, else any key\n";
	int firstUseInt = 0;
	cin >> firstUseInt;
	cout << "\n";
	if(firstUseInt ==1){
		firstUse = true;
		cout<< "Reset starts...\n";
	}
	else{
		firstUse = false;
		cout << "Reset disabled\n";
	}


	//Serial can be used for setting user position limits,
	//move power and step mode (e.g. wth = tilt half step mode)
	//wta/wpa is automatic step mode and leads to resolution of
	// 46.2857 rsp 11.5714 arc sec pp, tmr is tilt regular move power
	if (firstUse) {
		if (SerialStringOut(COMstream, (unsigned char *)"TMR WTA WPA ") != TRUE) {
			cout << "1st Serial command not sent to PTU \n";
			getchar();
		}
		// tilt reg move power, tilt min pos, t max pos, p m pos,
		// p x pos, user limits enabled, disable reset on restart,
		//t acc, p acc, p upper speed limit, t u s l
		if (SerialStringOut(COMstream, (unsigned char *)"TMR TNU-1000 TXU9000 PNU-3000 PXU3000 LU RD TA200000 PA50000 PU10000 TU10000 ") != TRUE) {
			cout << "2nd Serial command not sent to PTU \n";
		}
		cout << "Wait until PTU has stopped and then enter a key! \n";
		int dummivar = 0;
		cin>>dummivar;
		reset_PTU_parser(2000); // needed for changing between direct serial comm and cpi commands
								// set pure velocity mode
		if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) == PTU_OK) {
			cout << "Speed mode set to pure velocity \n";
		}
		maxPos = (short)get_current(PAN, MAXIMUM_POSITION);
		if (maxPos == 0) {
			cout << "could not set or receive max pan position limit, extremumpos:  \n" << maxPos;
		}
		maxPos = 0;
		maxPos = (short)get_current(PAN, MINIMUM_POSITION);
		if (maxPos == 0) {
			cout << "could not set or receive min pan position limit, extremumpos:  \n" << maxPos;
		}
		maxPos = 0;
		maxPos = (short)get_current(TILT, MINIMUM_POSITION);
		if (maxPos == 0) {
			cout << "could not set or receive min tilt position limit, extremumpos:  \n" << maxPos;
		}
		// lower base speed:
		// set base speed to 200 (minimum is 57/58)
		signed short int val = 58; // ^= 5deg/sec pan; 2.5deg/sec tilt base speed
		if (set_desired(PAN, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Pan base speed could not be set \n";
		}
		val = 4 * val;
		if (set_desired(TILT, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Pan base speed could not be set \n";
		}
		cout << "Base speed pan: " << (short)get_current(PAN, BASE) << "\n";
		cout << "Base speed tilt: " << (short)get_current(TILT, BASE) << "\n";
		//// acceleration
		//val = 200000;
		//if (set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Acceleration pan could not be set!\n";
		//}
		//if (set_desired(TILT, ACCELERATION, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Acceleration pan could not be set!\n";
		//}
		//// speed limits
		//val = 10000;
		//if (set_desired(PAN, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Upper speed limit pan could not be set!\n";
		//}
		//if (set_desired(TILT, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Upper speed limit pan could not be set!\n";
		//}
	}
	gettimeofday(&t1, NULL);
	ros::init(argc, argv, "ptuctrl_node");
	subAndpub SAPObject;
	ros::spin();


	// rehome PTU
	val = 1000;
	set_desired(PAN, SPEED, &val, ABSOLUTE);
	set_desired(TILT, SPEED, &val, ABSOLUTE);
	val = 0;
	set_desired(PAN, POSITION, &val, ABSOLUTE);
	val = 0;
	set_desired(TILT, POSITION, &val, ABSOLUTE);
	close_host_port(COMstream);				// close connection


	return 0;
}
