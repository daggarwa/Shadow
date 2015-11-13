#include "team_assignment_5/SerialComm.h"

#include <ros/ros.h>
#include <cstdlib>

int main (int argc, char **argv)
{
	ros::init (argc, argv, "create_controller");
	ros::NodeHandle n;
	ros::ServiceClient cli = n.serviceClient<team_assignment_5::SerialComm>("serial_comm");
	
	ROS_INFO("***********************************************");
	ROS_INFO("Note: Press arrow keys or 's' to control iRobot");
	ROS_INFO("Also, pressing ENTER ends the program...");
	ROS_INFO("***********************************************\n");
	std::string space = "                                ";
	
	team_assignment_5::SerialComm msg;
	while(1)
	{
		// get characters "cleanly" without waiting
		if (msg.request.button != 27 and msg.request.button != 91)
			std::cout << std::endl << space << "Please choose command: ";
		system ("/bin/stty raw");
		msg.request.button = std::cin.get();
		system ("/bin/stty cooked");
		if (msg.request.button != 27 and msg.request.button != 91)
			std::cout << std::endl;
		
		// end on pressing <ENTER>
		if (msg.request.button == 13) break;
		
		if (cli.call(msg))
		{ // *** alternatively *** also send another code to stop previous command
			if (msg.response.result < 1)
				ROS_INFO("request sent successfully");
			else if (msg.request.button != 27 and msg.request.button != 91)
				ROS_INFO("request sent, but rejected");
		}
		else ROS_ERROR("failed to call service serial_comm");
	}

	return 0;
}
