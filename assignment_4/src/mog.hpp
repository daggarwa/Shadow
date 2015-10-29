
	void MOG2()
	{
		cv::BackgroundSubtractorMOG2 bg (5, 36);
				
		sensor_msgs::ImagePtr out_frame;
		cv::Mat flow_th, flow_contours, fgMaskMOG2;
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;

		cv::Mat d_element = getStructuringElement (cv::MORPH_RECT, cv::Size(10, 10));
		cv::Mat e_element = getStructuringElement (cv::MORPH_RECT, cv::Size(3, 3));
		
		ros::Rate loop_rate(10);

		while (nh1_.ok() and choice == 2)
		{
			if (!current_grayframe.empty())
			{
				bg.operator() (current_grayframe, fgMaskMOG2);

				cv::threshold (fgMaskMOG2, flow_th, 50, 255, CV_THRESH_BINARY);
				
				cv::dilate (flow_th, flow_th, d_element, cv::Point (-1, -1), 3);
				cv::erode (flow_th, flow_th, e_element, cv::Point (-1, -1), 1);
				
				#include "bounding_boxes.hpp"

				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", current).toImageMsg();
				image_pub_.publish(out_frame);
			}
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
