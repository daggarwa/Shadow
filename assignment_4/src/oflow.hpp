
	void OpticalFlow()
	{
		double max;
		
		sensor_msgs::ImagePtr out_frame;
		cv::Mat rgb_flow, flow_th, flow_contours;
		
		cv::Mat flow_mag, flow_x, flow_y;
		
		cv::Mat d_element = getStructuringElement (cv::MORPH_ELLIPSE, cv::Size(10, 10));
		cv::Mat e_element = getStructuringElement (cv::MORPH_ELLIPSE, cv::Size(5, 5));

		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;

		ros::Rate loop_rate(10);

		while (nh_.ok())
		{
			if (!current_grayframe.empty())
			{
				// Operation on Flow Image 
				cv::calcOpticalFlowFarneback (prev_grayframe, current_grayframe,
					flow, 0.5, 1, 5, 3, 5, 1.1, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
				
				cv::vector<cv::Mat> planes(2);
				split (flow, planes);
				flow_x = planes[0];
				flow_y = planes[1];		
				magnitude (flow_x, flow_y, flow_mag);
				flow_mag.convertTo (flow_mag, CV_8U);

				int PATCH_SIZE = 15;
				int THRESH = 5;

				// ignore small (non-)movements
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

				// Morphological Operation on Thresholded Flow Image
				cv::dilate(flow_th, flow_th, d_element, cv::Point (-1, -1), 3);
				cv::erode(flow_th, flow_th, e_element, cv::Point (-1, -1), 1);
				
				#include "bounding_boxes.hpp"
						
				//cv::imshow (OPENCV_WINDOW, flow_th);
				//cv::imshow ("FLOW MAG", flow_mag);
				cv::imshow ("Motion Detection (Farneback)", current_grayframe);
				cv::waitKey(1);

				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", rgb_flow).toImageMsg();

				// output modified video stream
				image_pub_.publish(out_frame);
			}
			
			ros::spinOnce();
			loop_rate.sleep();
			
			std::swap (prev_grayframe, current_grayframe);
		}
	}
