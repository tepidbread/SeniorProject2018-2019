
//#include <ros/ros.h>
#include <iostream>
using namespace std;
#include <stdio.h>
#include <math.h>
#include <robotcontrol.h>

#define PI 3.141592654

//To Do:
//Add integral control
//Tune and update gain values, constants, and measurements
//ROS integration
//Multithread (if needed)
//General cleaning of c

void nextpose(double &x, double &y,double &theta, double v, double w, uint64_t Ts) //Call at the end of main loop
{
	//Using Runge-Kutta
	x = x + (v*(Ts * 1e-9)*cos(theta+((w*(Ts * 1e-9))/2)));
	y = y + (v*(Ts * 1e-9)*sin(theta+((w*(Ts * 1e-9))/2)));
	theta = theta + w*(Ts * 1e-9); //This needs to be [0,2pi]

	theta = fmod(theta,2*PI);
	/*
	if(theta > PI){
		theta = theta + -(2*PI);
	}
	else if(theta < -PI){
		theta = theta + (2*PI);
	}
	*/
}

void test(){
	//rc_encoder_eqep_init();
	int64_t tmp;
	tmp = rc_encoder_eqep_read(2);
	cout << "Encoder value is:\t" << tmp << endl;
}

int main(/* int argc, char **argv */){
/*
	//Initializing Ros Node
	//ros::init(argc, argv, "ROBOT_CONTROL_NODE")
	//ros::NodeHandle nh;

	//ROS_INFO_STREAM("THIS WORKS!");

	//Initialzation of constants
	double d = 0.08; //distance between wheels (not correct value)
	double r = 0.0342; //radius of wheels (not correct value)

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
	double ActualW = 0.00;
	double ActualV = 0.00;

	double CurrentX = 0.00;
	double CurrentY = 0.00;
	double CurrentTheta = 0.00;	//This needs to be set relative to some global referance frame initially


	double TargetX = 2;
	double TargetY = 1;
	double ThetaError = 0.00;

	double LineAngle = 0.00;	//Angle of line between target and current
	double LineDistance = 0.00;	//Distance of line between target and current
	double vIntegral = 0.00;
	double PrevvIntegral = 0.00;
	double PrevwIntegral = 0.00;
	double wIntegral = 0.00;

	double DesiredLeftWheelSpeed = 0.00;
	double DesiredRightWheelSpeed = 0.00;
	double RightWheelSpeedError = 0.00;
	double LeftWheelSpeedError = 0.00;
	double PrevLeftWheelSpeedError = 0.00;
	double PrevRightWheelSpeedError = 0.00;
	double CurrentLeftWheelSpeed = 0.00;
	double CurrentRightWheelSpeed = 0.00;

	double LeftWheelIntegral = 0.00;
	double RightWheelIntegral = 0.00;
	double PrevRightWheelIntegral = 0.00;
	double PrevLeftWheelIntegral = 0.00;

	double LeftWheelDerivative = 0.00;
	double RightWheelDerivative = 0.00;

	double LeftWheelCommand = 0.00;
	double RightWheelCommand = 0.00;

	int64_t LeftEncoderBeginCount = 0;
	int64_t RightEncoderBeginCount = 0;
	int64_t LeftEncoderEndCount = 0;
	int64_t RightEncoderEndCount = 0;

	uint64_t EncoderBeginTime = 0;
	uint64_t EncoderEndTime = 0;
	uint64_t DeltaT = 0;
	//====================================================================

	//Initialzation of RC functionality
	rc_motor_init(); //initializes all 4 motors into free spin state
	rc_encoder_eqep_init(); //initializes eqep encoder counters for channels 1-3
	//====================================================================

	while(1)
	{
			EncoderBeginTime = rc_nanos_since_epoch();	//Log Time at begining of code execution[ns]

			LeftEncoderBeginCount = rc_encoder_eqep_read(2);	//Reading Left wheel encoder value [encoder counts]
			RightEncoderBeginCount = rc_encoder_eqep_read(3);	//Reading Right wheel encoder value [encoder counts]
			cout << "left encoder begin is:" << "\t" << LeftEncoderBeginCount << endl;
			rc_nanosleep(Ts);

			EncoderEndTime = rc_nanos_since_epoch();	//Log Time at end of code execution[ns]

			LeftEncoderEndCount = rc_encoder_eqep_read(2);	//Reading Left wheel encoder value [encoder counts]
			RightEncoderEndCount = rc_encoder_eqep_read(3);	//Reading Right wheel encoder value	[encoder counts]
			cout << "left encoder end is:" << "t" << LeftEncoderEndCount << endl;
			DeltaT = EncoderEndTime-EncoderBeginTime;
			cout<<"DeltaT is:" << "\t" << DeltaT << endl;
			LineDistance = sqrt(pow((TargetX - CurrentX),2)+pow((TargetY - CurrentY),2));	//[m]
			vIntegral = (((double)(DeltaT)*1e-9)*LineDistance) + PrevvIntegral;
			PrevvIntegral = vIntegral;
			v = Kp_v*LineDistance+Ki_v*vIntegral;	//[m/s]


			LineAngle = atan2((TargetY - CurrentY),(TargetX - CurrentX));	//[0,2pi] radians
			ThetaError = LineAngle - CurrentTheta;	//Determining the robot's angle from the target point [radians]
			wIntegral = (((double)(DeltaT)*1e-9)*ThetaError) + PrevwIntegral;
			PrevwIntegral = wIntegral;
			w = Kp_gamma*ThetaError+Ki_gamma*wIntegral;	//Simple porportional control based on ThetaError and Steering Gain [radians/s]

			if(LineDistance < d){
				v = 0.0;
				w = 0.0;
			}
			if(v > 0.5){
				v = 0.5;
			}
			if(w > 5){
				w = 5;
			}

			DesiredLeftWheelSpeed = ((2*v)-(d*w))/2;	//DesiredLeftWheelSpeed based on linear and angular velocity command [m/s]
			DesiredRightWheelSpeed = -((2*v)+(d*w))/2;	//DesiredRightWheelSpeed based on linear and angular velocity command [m/s]


			//Determining the current wheel speeds to see if they match what is commanded of them (check type casting ) *some vars are double, int
			CurrentLeftWheelSpeed = ((2*PI*r)/60)*((((double)((LeftEncoderEndCount - LeftEncoderBeginCount))/((int64_t)(DeltaT)))) * ((double)(60000000000/2133)));	// [m/s]
			CurrentRightWheelSpeed = ((2*PI*r)/60)*((((double)((RightEncoderEndCount - RightEncoderBeginCount))/((int64_t)(DeltaT)))) * ((double)(60000000000/2133)));	// [m/s]

			ActualV = .5*((-CurrentRightWheelSpeed)+CurrentLeftWheelSpeed);	//Calculating the actual linear velocity based on current wheel speeds
			ActualW = (1/d)*((-CurrentRightWheelSpeed)-CurrentLeftWheelSpeed);	//Calculating the actual angular velocity based on current wheel speeds


			LeftWheelSpeedError = (DesiredLeftWheelSpeed - CurrentLeftWheelSpeed);	//Calculating difference between desired wheel speed and actual
			RightWheelSpeedError = (DesiredRightWheelSpeed - CurrentRightWheelSpeed);

			LeftWheelIntegral = (((double)(DeltaT)*1e-9)*LeftWheelSpeedError) + PrevLeftWheelIntegral;
			RightWheelIntegral = (((double)(DeltaT)*1e-9)*RightWheelSpeedError) + PrevRightWheelIntegral;

			PrevLeftWheelIntegral = LeftWheelIntegral;
			PrevRightWheelIntegral = RightWheelIntegral;

			PrevLeftWheelSpeedError = LeftWheelSpeedError;
			PrevRightWheelSpeedError = RightWheelSpeedError;

			LeftWheelDerivative = (LeftWheelSpeedError-PrevLeftWheelSpeedError)/DeltaT;
			RightWheelDerivative =(RightWheelSpeedError-PrevRightWheelSpeedError)/DeltaT;

			LeftWheelCommand = (Kp*(LeftWheelSpeedError))+(Ki*LeftWheelIntegral)-(Kd*LeftWheelDerivative);
			RightWheelCommand = (Kp*(RightWheelSpeedError))+(Ki*RightWheelIntegral)-(Kd*RightWheelDerivative);

			cout << "CurrentTheta\t" << CurrentTheta << endl << "LineAngle\t" << LineAngle << endl;
			cout << "CurrentX\t" << CurrentX << endl << "CurrentY\t" << CurrentY << endl;
			cout << "w\t" << w << endl << "Actual\t" << ActualW << endl << endl;

			rc_motor_set(2,LeftWheelCommand);	//Set channel 2 pulse width with proportional gain
			rc_motor_set(3,RightWheelCommand);	//Set channel 3 pulse width with proportional gain

			nextpose(CurrentX, CurrentY, CurrentTheta, ActualV, ActualW, Ts);

	}
*/	
	rc_motor_init();
	rc_encoder_eqep_init();
	while (1){
	test();
	}
}
