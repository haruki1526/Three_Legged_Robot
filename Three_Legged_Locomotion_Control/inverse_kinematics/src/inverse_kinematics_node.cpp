#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

#define L1 0.04
#define L2 0.1
#define L3 0.1




class control{
	public:
		void inverse_kinematics(double x, double y, double z);

		void right_command(double theta1, double theta2, double theta3);


	private:
		double calcu_theta1;
		
		double calcu_theta2;
		
		double calcu_theta3;

		ros::Publisher pub;

				


};

void control::right_command(double theta1, double theta2, double theta3){
	
	

	printf("%f %f %f\n",theta1,theta2,theta3);



}

void control::inverse_kinematics(double x, double y, double z){

	calcu_theta1=std::atan2(y, z);

	calcu_theta3=std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2) -L3*L3)/(2*L2*sqrt(x*x+std::pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2) ))) + 
		std::acos((x*x+std::pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2)+L3*L3-L2*L2)/ (2*L3*std::sqrt(x*x+std::pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2))));



	calcu_theta2=std::atan2(x, (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)))- 
		std::acos((L2*L2+x*x+std::pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2) -L3*L3)/(2*L2*std::sqrt(x*x+pow( (z-L1*std::cos(calcu_theta1))/(std::cos(calcu_theta1)), 2) )));
       	

	right_command(calcu_theta1, calcu_theta2, calcu_theta3);


}




int main(){

	control Control;

	Control.inverse_kinematics(0.1366025, 0.0883012, 0.15294);


}
