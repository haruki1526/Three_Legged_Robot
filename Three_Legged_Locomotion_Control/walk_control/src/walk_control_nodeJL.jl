#!/usr/bin/env julia

using RobotOS

@rosimport robo_state.msg: robo_command
rostypegen()

using robo_state.msg

const L1=0.04
const L2=0.1
const L3=0.1
function Inverse_kinematics(x, y, z)
	theta1=atan2(y, z)

	theta2=atan2(x,(z-L1*cos(theta1))/(cos(theta1)))-acos((L2^2+x^2+((z-L1*cos(theta1))/(cos(theta1)))^2 -L3*L3)/(2*L2 *sqrt(x^2+((z-L1*cos(theta1))/(cos(theta1)))^2)))

		
	

							      
	theta3=acos((L2^2+x^2+((z-L1*cos(theta1))/(cos(theta1)))^2 -L3^2)/(2*L2 *sqrt(x^2+((z-L1*cos(theta1))/(cos(theta1)))^2))) + acos((x^2+((z-L1*cos(theta1))/(cos(theta1)))^2+L3^2-L2^2)/(2*L3 *sqrt(x^2+((z-L1*cos(theta1))/(cos(theta1)))^2)))
	
	
	theta1, theta2, theta3
	
end

function pub_func(theta1, theta2, theta3, pub)
	positions = robo_command()
	positions. = 
	
	end

end

function main()
	init_node("walk_controller")
	pub = Publisher{robo_command}("trajectory_command", queue_size=10)
	x = 0.1366025
	y = 0.088012
	z = 0.15294
	theta1, theta2, theta3 = Inverse_kinematics(x, y, z)
	pub_func(theta1, theta2, theta3, pub)
	
end

main()
