
	void OpticalFlow()
	{
		double max;
		
		sensor_msgs::ImagePtr out_frame;
		cv::Mat flow_th, flow_contours, prev_grayframe, flow_mag;
		cv::vector<cv::Mat> planes(2);
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		
		cv::Mat d_element = getStructuringElement (cv::MORPH_ELLIPSE, cv::Size(10, 10));
		cv::Mat e_element = getStructuringElement (cv::MORPH_ELLIPSE, cv::Size(5, 5));
		
		ros::Rate loop_rate(10);

		while (nh1_.ok() and choice == 1)
		{
			if (!current_grayframe.empty() and !prev_grayframe.empty())
			{
				cv::calcOpticalFlowFarneback (prev_grayframe, current_grayframe,
						flow, 0.5, 1, 5, 3, 5, 1.1, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
				
				split (flow, planes);
				magnitude (planes[0], planes[1], flow_mag);
				flow_mag.convertTo (flow_mag, CV_8U);

				// ignore small (non-)movements
				int PATCH_SIZE = 15, THRESH = 5;
				for (int i=PATCH_SIZE; i<flow_mag.rows-PATCH_SIZE; i++)
					for (int j=PATCH_SIZE; j<flow_mag.cols-PATCH_SIZE; j++)
						if (flow_mag.at<uchar> (i, j) > THRESH)
						{
							cv::Mat roi = flow_mag (cv::Rect (j, i, PATCH_SIZE, PATCH_SIZE));
							cv::minMaxLoc (roi, NULL, &max);
							if (max < 20)
								flow_mag.at<uchar> (i, j) = 0;
						}
				
				cv::threshold (flow_mag, flow_th, THRESH, 255, CV_THRESH_BINARY);

				cv::dilate(flow_th, flow_th, d_element, cv::Point (-1, -1), 3);
				cv::erode(flow_th, flow_th, e_element, cv::Point (-1, -1), 1);
				
				#include "bounding_boxes.hpp"
						
				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", current).toImageMsg();
				image_pub_.publish(out_frame);
			}
			
			ros::spinOnce();
			loop_rate.sleep();
			
			std::swap (prev_grayframe, current_grayframe);
		}
	}
