//#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <stdio.h>
#include <thread>
using namespace std;
#include <mutex>
using namespace std;
#include <math.h>
#include <robotcontrol.h>

#define PI 3.141592654

//To Do:
//Add integral control
//Tune and update gain values, constants, and measurements
//ROS integration
//Multithread (if needed)
//General cleaning of code

mutex pose; //mutex test


void nextpose(double *x, double *y,double *theta, double *v, double *w, uint64_t Ts) //Call at the end of main loop
{ double tmpX,tmpY,tmpTheta,tmpV,tmpW;
	//No mutex needed main does not write to these
	tmpX = *x;
	tmpY = *y;
	tmpTheta = *theta;

	while(1){

		pose.lock();
			*x = tmpX;
			*y = tmpY;
			*theta = tmpTheta;
			tmpV = *v;
			tmpW = *w;
		pose.unlock();

		//Using Runge-Kutta
		tmpX = tmpX + (tmpV*(Ts * 1e-9)*cos(tmpTheta+((tmpW*(Ts * 1e-9))/2)));
		tmpY = tmpY + (tmpV*(Ts * 1e-9)*sin(tmpTheta+((tmpW*(Ts * 1e-9))/2)));
		tmpTheta = tmpTheta + tmpW*(Ts * 1e-9); //This needs to be [0,2pi]

		tmpTheta = fmod(tmpTheta,2*PI);
		/*
		if(theta > PI){
			theta = theta + -(2*PI);
		}
		else if(theta < -PI){
			theta = theta + (2*PI);
		}
		*/
	}
}

int main(/* int argc, char **argv */){

	//Initializing Ros Node
	//ros::init(argc, argv, "ROBOT_CONTROL_NODE")
	//ros::NodeHandle nh;

	//ROS_INFO_STREAM("THIS WORKS!");

	//Initialzation of constants
	double d = 0.0675; //distance between wheels (not correct value)
	double r = 0.03; //radius of wheels (not correct value)

	double Kp_gamma = 5;
	double Ki_gamma = 0.5;

	double Kp_v = 0.5;
	double Ki_v = 0.5;

	double Kp = 1.85;//1.85;
	double Ki = 2;
	double Kd = 0.0185;

	uint64_t Ts = 5000000; //[ns]


	//====================================================================
	//Initialzation of varibles that will change upon runtime
	double w = 0.00;	//radial velocity
	double v = 0.00;	//linear velocity (constant value for now)
	double actualW = 0.00;
	double actualV = 0.00;

	double currentX = 0.00;
	double currentY = 0.00;
	double currentTheta = 0.00;	//This needs to be set relative to some global referance frame initially


	double targetX = 2;
	double targetY = 1;
	double thetaError = 0.00;

	double lineAngle = 0.00;	//Angle of line between target and current
	double lineDistance = 0.00;	//Distance of line between target and current
	double vIntegral = 0.00;
	double prevVIntegral = 0.00;
	double prevWIntegral = 0.00;
	double wIntegral = 0.00;

	double desiredLeftWheelSpeed = 0.00;
	double desiredRightWheelSpeed = 0.00;
	double rightWheelSpeedError = 0.00;
	double leftWheelSpeedError = 0.00;
	double prevLeftWheelSpeedError = 0.00;
	double prevRightWheelSpeedError = 0.00;
	double currentLeftWheelSpeed = 0.00;
	double currentRightWheelSpeed = 0.00;

	double leftWheelIntegral = 0.00;
	double rightWheelIntegral = 0.00;
	double prevRightWheelIntegral = 0.00;
	double prevLeftWheelIntegral = 0.00;

	double leftWheelDerivative = 0.00;
	double rightWheelDerivative = 0.00;

	double leftWheelCommand = 0.00;
	double rightWheelCommand = 0.00;

	int64_t leftEncoderBeginCount = 0;
	int64_t rightEncoderBeginCount = 0;
	int64_t leftEncoderEndCount = 0;
	int64_t rightEncoderEndCount = 0;

	uint64_t encoderBeginTime = 0;
	uint64_t encoderEndTime = 0;
	uint64_t deltaT = 0;
	//====================================================================

	//Initialzation of RC functionality
	rc_motor_init(); //initializes all 4 motors into free spin state
	rc_encoder_eqep_init(); //initializes eqep encoder counters for channels 1-3
	//====================================================================

		thread deadReckoning(nextpose,&currentX,&currentY,&currentTheta,&actualV,&actualW,Ts);
		deadReckoning.detach();

	while(1)
	{
			encoderBeginTime = rc_nanos_since_epoch();	//Log Time at begining of code execution[ns]

			leftEncoderBeginCount = rc_encoder_eqep_read(2);	//Reading Left wheel encoder value [encoder counts]
			rightEncoderBeginCount = rc_encoder_eqep_read(3);	//Reading Right wheel encoder value [encoder counts]

			rc_nanosleep(Ts);

			encoderEndTime = rc_nanos_since_epoch();	//Log Time at end of code execution[ns]

			leftEncoderEndCount = rc_encoder_eqep_read(2);	//Reading Left wheel encoder value [encoder counts]
			rightEncoderEndCount = rc_encoder_eqep_read(3);	//Reading Right wheel encoder value	[encoder counts]

			deltaT = encoderEndTime-encoderBeginTime;

			lineDistance = sqrt(pow((targetX - currentX),2)+pow((targetY - currentY),2));	//[m]
			vIntegral = (((double)(deltaT)*1e-9)*lineDistance) + prevVIntegral;
			prevVIntegral = vIntegral;
			v = Kp_v*lineDistance+Ki_v*vIntegral;	//[m/s]


			lineAngle = atan2((targetY - currentY),(targetX - currentX));	//[0,2pi] radians
			thetaError = lineAngle - currentTheta;	//Determining the robot's angle from the target point [radians]
			wIntegral = (((double)(deltaT)*1e-9)*thetaError) + prevWIntegral;
			prevWIntegral = wIntegral;
			w = Kp_gamma*thetaError+Ki_gamma*wIntegral;	//Simple porportional control based on thetaError and Steering Gain [radians/s]

			if(lineDistance < d){
				v = 0.0;
				w = 0.0;
			}
			if(v > 0.5){
				v = 0.5;
			}
			if(w > 5){
				w = 5;
			}

			desiredLeftWheelSpeed = ((2*v)-(d*w))/2;	//desiredLeftWheelSpeed based on linear and angular velocity command [m/s]
			desiredRightWheelSpeed = -((2*v)+(d*w))/2;	//desiredRightWheelSpeed based on linear and angular velocity command [m/s]


			//Determining the current wheel speeds to see if they match what is commanded of them (check type casting ) *some vars are double, int
			currentLeftWheelSpeed = ((2*PI*r)/60)*((((double)((leftEncoderEndCount - leftEncoderBeginCount))/((int64_t)(deltaT)))) * ((double)(60000000000/2133)));	// [m/s]
			currentRightWheelSpeed = ((2*PI*r)/60)*((((double)((rightEncoderEndCount - rightEncoderBeginCount))/((int64_t)(deltaT)))) * ((double)(60000000000/2133)));	// [m/s]

			actualV = .5*((-currentRightWheelSpeed)+currentLeftWheelSpeed);	//Calculating the actual linear velocity based on current wheel speeds
			actualW = (1/d)*((-currentRightWheelSpeed)-currentLeftWheelSpeed);	//Calculating the actual angular velocity based on current wheel speeds


			leftWheelSpeedError = (desiredLeftWheelSpeed - currentLeftWheelSpeed);	//Calculating difference between desired wheel speed and actual
			rightWheelSpeedError = (desiredRightWheelSpeed - currentRightWheelSpeed);

			leftWheelIntegral = (((double)(deltaT)*1e-9)*leftWheelSpeedError) + prevLeftWheelIntegral;
			rightWheelIntegral = (((double)(deltaT)*1e-9)*rightWheelSpeedError) + prevRightWheelIntegral;

			prevLeftWheelIntegral = leftWheelIntegral;
			prevRightWheelIntegral = rightWheelIntegral;

			prevLeftWheelSpeedError = leftWheelSpeedError;
			prevRightWheelSpeedError = rightWheelSpeedError;

			leftWheelDerivative = (leftWheelSpeedError-prevLeftWheelSpeedError)/deltaT;
			rightWheelDerivative =(rightWheelSpeedError-prevRightWheelSpeedError)/deltaT;

			leftWheelCommand = (Kp*(leftWheelSpeedError))+(Ki*leftWheelIntegral)-(Kd*leftWheelDerivative);
			rightWheelCommand = (Kp*(rightWheelSpeedError))+(Ki*rightWheelIntegral)-(Kd*rightWheelDerivative);

			cout << "currentTheta\t" << currentTheta << endl << "lineAngle\t" << lineAngle << endl;
			cout << "currentX\t" << currentX << endl << "currentY\t" << currentY << endl;
			cout << "w\t" << w << endl << "Actual\t" << actualW << endl << endl;

			rc_motor_set(2,leftWheelCommand);	//Set channel 2 pulse width with proportional gain
			rc_motor_set(3,rightWheelCommand);	//Set channel 3 pulse width with proportional gain

	}
	return 1;
}
