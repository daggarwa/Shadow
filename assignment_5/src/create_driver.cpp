#include "team_assignment_5/SerialComm.h"

#include <ros/ros.h>
#include <string.h>
#include <termios.h>

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#define VELOCITYCHANGE 200
#define ROTATIONCHANGE 150

int fd; // serial port

int move (int vl, int vr)
{
	int opcode = 145;
	int m = 256*256*vl+vr;
	if (write (fd, &opcode, 1) < 0)
		return 1;
	if (write (fd, &m, 4) < 0)
		return 1;
	usleep (120*1000);
	m = 0; // and stop soon (120 ms)
	if (write (fd, &opcode, 1) < 0)
		return 1;
	if (write (fd, &m, 4) < 0)
		return 1;
}

bool callback (team_assignment_5::SerialComm::Request &req,
		team_assignment_5::SerialComm::Response &response)
{
	int vr = 0, vl = 0, opcode, note;
	response.result = -1;
	switch (req.button)
	{
		case 65: // UP
			ROS_INFO("move ahead");
			vr = -VELOCITYCHANGE, vl = -VELOCITYCHANGE;
			break;
		case 66: // DOWN
			ROS_INFO("move back");
			vr = VELOCITYCHANGE*1.2, vl = VELOCITYCHANGE*1.2;
			break;
		case 67: // RIGHT
			ROS_INFO("rotate right");
			vr = ROTATIONCHANGE, vl = -ROTATIONCHANGE;
			break;
		case 68: // LEFT
			ROS_INFO("rotate left");
			vr = -ROTATIONCHANGE, vl = ROTATIONCHANGE;
			break;
		case 115:
			ROS_INFO("play sound");
			response.result = 0;
			opcode = 140, note = 3;
			write (fd, &opcode, 1);
			write (fd, &note, 1); // loop later
			note = 1; write (fd, &note, 1);
			note = 64; write (fd, &note, 1);
			note = 16; write (fd, &note, 1);
			note = 141; write (fd, &note, 1);
			note = 3; write (fd, &note, 1);
			break;
		default:
			response.result = 1;
			break;
	}
	
	int r = move (vl, vr);
	if (response.result < 1)
	{
		if (response.result == 0 or r != 1)
			ROS_INFO("service request accepted");
		else ROS_INFO("failed"); // disconnect, maybe
	}
	else if (req.button != 27 and req.button != 91)
		ROS_INFO("service request ignored");
	
	return true;
}

int main (int argc, char** argv)
{	
	ros::init (argc, argv, "create_driver");	
	ros::NodeHandle n;
	ros::ServiceServer ser = n.advertiseService ("serial_comm", callback);
	
	#include "port_stuff.h"
	
	while (n.ok())
	{
		ros::spinOnce();
	}
	
	// won't ever happen -_-
	close(fd);
	
	return 0;
}
