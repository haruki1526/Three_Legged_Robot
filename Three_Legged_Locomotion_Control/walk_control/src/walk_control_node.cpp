#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <robo_state/robo_command.h>

#define L1 0.04
#define L2 0.1
#define L3 0.1

class inverse_kinematics{
	public:


		double theta1_calculater(double y, double z);

		double theta2_calculater(double x, double z);
		
		double theta3_calculater(double x, double z);


	private:
		double theta1;

		double theta2;

		double theta3;


};

class control{
	public:

		void command(double position[3][3]);


	private:

		inverse_kinematics Inverse_kinematics;		

		
		robo_state::robo_command commander;

		ros::NodeHandle node;

		
		ros::Publisher pub= node.advertise<robo_state::robo_command>("position_command",5);



};


inline	double inverse_kinematics::theta1_calculater(double y, double z){


	theta1=std::atan2(y, z);

	return theta1;

}

inline	double inverse_kinematics::theta2_calculater(double x, double z){

	theta2=std::atan2(x, (z-L1*std::cos(theta1))/(std::cos(theta1)))- 
	std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) -L3*L3)/(2*L2*std::sqrt(x*x+pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) )));

	return theta2;
}

inline	double inverse_kinematics::theta3_calculater(double x, double z){


	theta3=std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) -L3*L3)/(2*L2*sqrt(x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2) ))) + 
		std::acos((x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2)+L3*L3-L2*L2)/ (2*L3*std::sqrt(x*x+std::pow( (z-L1*std::cos(theta1))/(std::cos(theta1)), 2))));

	return theta3;
}

void control::command(double position[3][3]){


	int i;
	for( i=0; i<3; i++){

		commander.theta.push_back(Inverse_kinematics.theta1_calculater(position[i][1], position[i][2]));

		commander.theta.push_back(Inverse_kinematics.theta2_calculater(position[i][0], position[i][2]));

		commander.theta.push_back(Inverse_kinematics.theta3_calculater(position[i][0], position[i][2]));
	}


	pub.publish(commander);
	commander.theta.clear();

}


int main(int argc, char **argv){


		
	ros::init(argc, argv, "robo_controller");


	control legg;

	double position[3][3]={{-0.03, 0, 0.2},     //0.1366025 0.088012, 0.15294
			       {-0.03, 0, 0.2},  
			       {-0.03, 0, 0.2}};


	ros::Rate rate(1000);
	while(ros::ok()){	

		legg.command(position);
		rate.sleep();

	}


}


