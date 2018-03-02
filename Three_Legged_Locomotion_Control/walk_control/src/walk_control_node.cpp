//#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

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

void control::command(double position[3][3]){


	int i;
	for( i=0; i<3; i++){
		printf("theta1=%lf\n",Inverse_kinematics.theta1_calculater(position[i][1], position[i][2]));

		printf("theta2=%lf\n",Inverse_kinematics.theta2_calculater(position[i][0], position[i][2]));

		printf("theta3=%lf\n",Inverse_kinematics.theta3_calculater(position[i][0], position[i][2]));
	}

}


int main(int argc, char **argv){


//	ros::nodehandle node;
	
//	ros::init(argc, argv, "robo_controller");


	control legg;

	double position[3][3]={{0, 0, 0.15294},     //0.1366025 0.088012, 0.15294
			       {0.1366025, 0.088012, 0.15294},  
			       {0.1366025, 0.088012, 0.15294}};

	while(1){	

		legg.command(position);

	}


}


