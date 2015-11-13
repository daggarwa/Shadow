fd = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
if (fd == -1)
	ROS_ERROR("could not connect to /dev/ttyUSB0");
else
	ROS_INFO("connected to /dev/tty/USB0");
struct termios options;
if (tcgetattr (fd, &options) < 0)
{
    ROS_ERROR("could not GET current port options :(");
    return 1;
}
cfsetispeed (&options, B115200);
cfsetospeed (&options, B115200);
options.c_cflag &= ~PARENB;
options.c_cflag &= ~CSTOPB;
options.c_cflag |=  CS8;
options.c_cflag &= ~CRTSCTS;
if(tcsetattr(fd, TCSANOW, &options) < 0){
    ROS_ERROR("could not SET current port options");
    return 1;
}
