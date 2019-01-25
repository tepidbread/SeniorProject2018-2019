#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <robotcontrol.h>

#include <vector>
#include <fstream>
using namespace std;

#define PI 3.141592654

double myWheelSpeeds[2];

void messageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	int i = 0;

	for(std::vector<double>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it) {
		myWheelSpeeds[i] = *it;
		i++;
	}
	return;
}

double setDuty(double vError, double prevVError, double integralError, uint64_t Ts) //Wheel Speed Error,Previous Wheel Speed Error, time elapsed between calculations
{
double kp = 1.75; //1.75	//Proportional gain
double kd = 5; //0	//Derivative gain
double ki = 12.5; //10	//Integral gain

double dutyCycle = (kp*vError)+(ki*((vError + integralError)*Ts*1e-9))+(kd*((vError - prevVError)/Ts*1e-9));

return dutyCycle;
}


void calcPoseRK(double &x, double &y,double &theta, double leftWheelSpeed, double rightWheelSpeed, uint64_t Ts) 
{	//Function called to calculate the current Pose of the robot

	double d = 0.0825; 		//Distance between robot wheels
	double vActual = (0.5)*((rightWheelSpeed)+(leftWheelSpeed));			//Calculate actual linear velocity
	double wActual = (1/d)*((rightWheelSpeed)-(leftWheelSpeed));		//Calculate actual angular velocity

	//Using Runge-Kutta method:
	x = x + (vActual*(Ts * 1e-9)*cos(theta+((wActual*(Ts * 1e-9))/2)));
	y = y + (vActual*(Ts * 1e-9)*sin(theta+((wActual*(Ts * 1e-9))/2)));
	theta = theta + wActual*(Ts * 1e-9); 			//theta must be [0,2pi]

	if(theta > PI){
		theta = theta + -(2*PI);
	}
	else if(theta < -PI){
		theta = theta + (2*PI);
	}

}

void calcPoseEuler(double &x, double &y, double &theta, double leftWheelSpeed, double rightWheelSpeed, uint64_t Ts)
{
	double d = 0.065;
	double vActual = (0.5)*((rightWheelSpeed)+(leftWheelSpeed));
	double wActual = (1/d)*((rightWheelSpeed)-(leftWheelSpeed));

	x = x + (Ts*1e-9)*(vActual*cos(theta));
	y = y + (Ts*1e-9)*(vActual*sin(theta));
	theta = theta + (Ts*1e-9)*wActual;

	if (theta > PI){
		theta = theta + -(2*PI);
	}
	else if (theta < -PI){
		theta = theta + (2*PI);
	}

}

int main (int argc, char **argv)
{
	//====================================================================
	//=================  Create and Initialize ROS Node  =================
	//====================================================================
	ros::init(argc, argv, "lineFollowBBB");
	ros::NodeHandle nh;

	//====================================================================
	//==================  Declaration of ROS Subscriber  =================
	//====================================================================
	ros::Subscriber sub = nh.subscribe<std_msgs::Float64MultiArray>("desiredSpeed", 5, messageCallback);

	//====================================================================
	//==================  Declaration of ROS Publisher  ==================
	//====================================================================
	ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("poseBBB", 5);

	//====================================================================
	//================== Setup file for logging ==========================
	//====================================================================
//	ofstream outputFile;
//	outputFile.open ("logs.txt");
//	outputFile << "rightWheelSpeed\t\t" << "leftWheelSpeed\t\t" << "theta\t\t" << "Left Encoder\t\t" << "Right Encoder\n";

	//====================================================================
	//==================  Initialization of Constants  ===================
	//====================================================================
	double r = 0.035; //formerly 0.03 the new measurement is more correct					//Radius of wheels

	//uint64_t Ts = 10000000; 					// in [ns]
	uint64_t Ts = 2500000;

	//====================================================================
	//=======  Initialization of Variables that Change on Runtime  =======
	//====================================================================

	double startTime = 0.00;
	double leftEncEnd = 0.00;
	double leftEncStart = 0.00;
	double rightEncEnd = 0.00;
	double rightEncStart = 0.00;
	double endTime = 0.00;

	double currentX = 0.90;
	double currentY = 0.30;
	double currentTheta = PI/2;				//This needs to be set relative to some global referance frame initially

	double currentLeftWheelSpeed = 0.00;
	double currentRightWheelSpeed = 0.00;
	double currentRightSpeedError = 0.00;
	double currentLeftSpeedError = 0.00;
	double prevRightSpeedError = 0.00;
	double prevLeftSpeedError = 0.00;
	double integralErrorLeft = 0.00;
	double integralErrorRight = 0.00;

	double dutyLeft = 0.00;
	double dutyRight = 0.00;
	double timeElapsed = Ts;	//Set to Ts for the first iteration to avoid NaN plague.
	double deltaLeftEncoder = 0.00;
	double deltaRightEncoder = 0.00;

	//int i = 0;
	uint64_t t0 = 0.00;
	//====================================================================
	//==========  Initialization of librobotcontrol Functions  ===========
	//====================================================================
	rc_encoder_eqep_init();
	rc_motor_init();
	//rc_motor_init_freq(25000); //If you want to specify pwm frequency.

	//Set looping rate
	//ros::Rate rate(150);		//Looping at 150Hz


	t0 = rc_nanos_since_epoch();
//	while (i < 1500)
	while (ros::ok())
	{
		//Create and fill publisher message
		std_msgs::Float64MultiArray msgPose;
		msgPose.data.clear();

		//===========================================================
		//=======================  Loop Timing  =====================
		//===========================================================
		startTime = rc_nanos_since_epoch();		//TIC
		leftEncStart = rc_encoder_eqep_read(2);
		rightEncStart = rc_encoder_eqep_read(3);	//Negative Values Correspond With Positive Direction


		//Calculate current wheel speeds to compare to desired
		currentLeftWheelSpeed = (double)((2*PI*r)*(((double)(deltaLeftEncoder))/((double)(timeElapsed)))*(((double)(1000000000))/((double)(2133))));
		currentRightWheelSpeed = (double)((2*PI*r)*(((double)(-deltaRightEncoder))/((double)(timeElapsed)))*(((double)(1000000000))/((double)(2133))));

		//Pass the pose variables to the calcPose function
		calcPoseRK(currentX,currentY,currentTheta,currentLeftWheelSpeed,currentRightWheelSpeed, timeElapsed);
		//calcPoseEuler(currentX,currentY,currentTheta,currentLeftWheelSpeed,currentRightWheelSpeed,timeElapsed);

		//Calculate the velocity error to send to duty calc function
		currentLeftSpeedError = (myWheelSpeeds[0] - currentLeftWheelSpeed);
		currentRightSpeedError = (myWheelSpeeds[1] - currentRightWheelSpeed);

		//Set duty cycle
		dutyLeft = setDuty(currentLeftSpeedError, prevLeftSpeedError, integralErrorLeft, timeElapsed);
		dutyRight = -setDuty(currentRightSpeedError, prevRightSpeedError, integralErrorRight, timeElapsed);

//		outputFile << currentRightWheelSpeed << "\t\t" << currentLeftWheelSpeed << "\t\t" << currentTheta << "\t\t" << leftEncStart << "\t\t" << rightEncStart << endl;

		rc_motor_set(2, dutyLeft);
		rc_motor_set(3, dutyRight);

		//Populate the msgPose with x,y,theta from function
		double myPose[3] = {currentX, currentY, currentTheta};
		for (int i=0; i < 3; i++) {
		msgPose.data.push_back(myPose[i]);
		}

		//Publish robots current pose
		pub.publish(msgPose);

		//Set looping variables
		integralErrorLeft = integralErrorLeft+currentLeftSpeedError;
		integralErrorRight = integralErrorRight+currentRightSpeedError;
		prevLeftSpeedError = currentLeftSpeedError;
		prevRightSpeedError = currentRightSpeedError;


		rc_nanosleep(Ts); 						//Wait

		leftEncEnd = rc_encoder_eqep_read(2);
		rightEncEnd = rc_encoder_eqep_read(3);
		endTime = rc_nanos_since_epoch();		//TIC

		deltaLeftEncoder = leftEncEnd - leftEncStart;
		deltaRightEncoder = rightEncEnd - rightEncStart;
		timeElapsed = endTime - startTime;

		//i = i + 1;
		ros::spinOnce();

	}
//	outputFile.close();
}
