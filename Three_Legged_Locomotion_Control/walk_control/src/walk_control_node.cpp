//#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

#define L1 0.04
#define L2 0.1
#define L3 0.1

class inverse_kinematics{
	public:


		void theta1_calculater(double y, double z);

		void theta2_calculater(double x, double z);
		
		void theta3_calculater(double x, double z);


	private:
		double theta1;

		double theta2;

		double theta3;


};

class control{
	public:

		void command(double x, double y, double z);

	private:

		inverse_kinematics Inverse_kinematics;		

//		ros::Publisher pub;

//		robo_state::robo_command commander;


};


double inverse_kinematics::theta1_calculater(double y, double z){


	theta1=std::atan2(y, z);

	return theta1;

}

double inverse_kinematics::theta2_calculater(double x, double z){

	theta2=std::atan2(x, (z-L1*std::cos(theta1))/(std::cos(theta1)))- 
	std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) -L3*L3)/(2*L2*std::sqrt(x*x+pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) )));

	return theta2;
}

double inverse_kinematics::theta3_calculater(double x, double z){


	theta3=std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) -L3*L3)/(2*L2*sqrt(x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) ))) + 
		std::acos((x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2)+L3*L3-L2*L2)/ (2*L3*std::sqrt(x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2))));

	return theta3;
}

void control::command(double position[]){
	

	Inverse_kinematics.theta1_calculater(y, z);

	Inverse_kinematics.theta2_calculater(x, z);

	Inverse_kinematics.theta3_calculater(x, z);


}


int main(int argc, char **argv){


//	ros::nodehandle node;
	
//	ros::init(argc, argv, "robo_controller");


	control right;

	control left;

	control center;

	double position[10];

	while(1){	

		right.command(0.1366025, 0.088012, 0.15294);

		left.command(0.1366025, 0.088012, 0.15294);

		center.command(0.1366025, 0.088012, 0.15294);
	}


}


