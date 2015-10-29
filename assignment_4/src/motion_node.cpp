#include "a53094896_assignment_4/MvK.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string.h>

int choice;

bool change_mode (a53094896_assignment_4::MvK::Request &req,
		a53094896_assignment_4::MvK::Response &res)
{
	res.o = 0; // all okay
	if (req.mode == 0 or req.mode == 1 or req.mode == 2) choice = req.mode;
	if (req.mode == 0) ROS_INFO("requested mode: raw video");
	else if (req.mode == 1) ROS_INFO("requested mode: farneback");
	else if (req.mode == 2) ROS_INFO("requested mode: MOG2");
	else
	{
		ROS_INFO("requested mode: unknown");
		res.o = 1; // "error"
		choice = 0;
	}
	
	if (res.o == 0) ROS_INFO("service request accepted");
	else ROS_INFO("service request rejected; reverting to raw video");
	
	return true;
}

class ImageConverter
{
	private:

	ros::NodeHandle nh1_;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	cv::Mat current, current_grayframe, flow;

	public:

	ImageConverter() : it_(nh1_)
	{	
		// subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe ("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise ("/motion_node/output", 1);
	}

	void imageCb (const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
			current = cv_ptr->image;
			cvtColor (current, current_grayframe, CV_BGR2GRAY);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR ("cv_bridge exception: %s", e.what());
			return;
		}
	}

	// pardon the poor code
	// separation for now...
	#include "rawfeed.hpp"
	#include "mog.hpp"
	#include "oflow.hpp"
	
	static void drawOptFlowMap (const cv::Mat& flow, cv::Mat& cflowmap, int step)
	{
		for (int y=0; y<cflowmap.rows; y+=step)
			for (int x=0; x<cflowmap.cols; x+=step)
			{
				const cv::Point2f& fxy = flow.at<cv::Point2f> (y, x);
				cv::line (cflowmap, cv::Point (x, y), cv::Point (cvRound (x+fxy.x),
						cvRound (y+fxy.y)), CV_RGB (0, 255, 0));
				cv::circle (cflowmap, cv::Point (x, y), 2, CV_RGB (0, 255, 0), -1);
			}
	}
};

int main (int argc, char** argv)
{	
	ros::init (argc, argv, "motion_node");
	ImageConverter ic;
	
	ros::NodeHandle nh2;
	ros::ServiceServer ser = nh2.advertiseService("motion_mode_kb", change_mode);
	
	choice = 1;
	while(1)
	{
		if (choice == 0)
			ic.RawVideo();
		else if (choice == 1)
			ic.OpticalFlow();
		else if (choice == 2)
			ic.MOG2();
		else ROS_INFO("error: incorrent choice");
	}
	
	return 0;
}
