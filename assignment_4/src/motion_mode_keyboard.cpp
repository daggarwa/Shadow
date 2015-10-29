#include "ros/ros.h"
#include "a53094896_assignment_4/MvK.h"
#include <cstdlib>

int main (int argc, char **argv)
{
	ros::init (argc, argv, "motion_mode_keyboard");
	ros::NodeHandle n;
	ros::ServiceClient cli = n.serviceClient<a53094896_assignment_4::MvK>("motion_mode_kb");
	a53094896_assignment_4::MvK s;

	ROS_INFO("");
	ROS_INFO("***********************************************");
	ROS_INFO("Note: Enter mode# 0/1/2 for Raw/Farneback/MOG2.");
	ROS_INFO("***********************************************");

	while(1)
	{
		std::cout << "\n                                Please choose video mode: ";
		std::cin >> s.request.mode;
		if (cli.call(s))
		{
			if (s.response.o == 0) ROS_INFO("request sent successfully");
			else ROS_INFO("request sent but rejected");
		}
		else
		{ // (request could not be sent)
			ROS_ERROR("failed to call service motion_mode_kb");
			// return 1;
		}
	}

	return 0;
}
