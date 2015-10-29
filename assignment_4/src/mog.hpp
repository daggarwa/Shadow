
	void MOG2BackgroundSubtraction()
	{
		cv::BackgroundSubtractorMOG2 bg (5, 100);
				
		sensor_msgs::ImagePtr out_frame;
		cv::Mat rgb_flow, flow_th, flow_contours;

		cv::Mat fgMaskMOG2;

		cv::Mat d_element = getStructuringElement (cv::MORPH_RECT, cv::Size(10, 10));
		cv::Mat e_element = getStructuringElement (cv::MORPH_RECT, cv::Size(3, 3));

		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		
		ros::Rate loop_rate(10);

		while (nh_.ok())
		{
			if (!current_grayframe.empty())
			{
				bg.operator() (current_grayframe, fgMaskMOG2);

				cv::threshold (fgMaskMOG2, flow_th, 50, 255, CV_THRESH_BINARY);
				
				cv::dilate (flow_th, flow_th, d_element, cv::Point (-1, -1), 3);
				cv::erode (flow_th, flow_th, e_element, cv::Point (-1, -1), 1);
				
				#include "bounding_boxes.hpp"

				//cv::imshow (OPENCV_WINDOW, fgMaskMOG2);
				//cv::imshow ("THRESHOLD", flow_th);
				cv::imshow ("Motion (Background Subtration)", current_grayframe);
				cv::waitKey(1);

				// cv_bridge::CvImage out_msg;
				// out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
				// out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
				// out_msg.image	= sal_float_image; // Your cv::Mat
		
				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", rgb_flow).toImageMsg();

				// output modified video stream
				image_pub_.publish(out_frame);
			}
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
