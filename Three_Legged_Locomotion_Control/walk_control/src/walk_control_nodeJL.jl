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
	
	
	[theta1, theta2, theta3]
	
end

function pub_func(theta_command, pub)

	positions = robo_command()
	positions.theta = theta_command
	publish(pub, positions)	
	
end

function main()
	init_node("walk_controller")
	loop_rate = Rate(5)
	pub = Publisher{robo_command}("trajectory_command", queue_size=10)
	while ! is_shutdown()
		x1 = 0.1366025
		y1 = 0.088012
		z1 = 0.15294
		x2 = 0.1366025
		y2 = 0.088012
		z2 = 0.15294
		x3 = 0.1366025
		y3 = 0.088012
		z3 = 0.15294


		theta_command=append!( Inverse_kinematics(x1, y1, z1), Inverse_kinematics(x2, y2, z2))
		theta_command=append!(theta_command , Inverse_kinematics(x3, y3, z3))

	
		pub_func(theta_command, pub)
		rossleep(loop_rate)

	end
	
end

main()
