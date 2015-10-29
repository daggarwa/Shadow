
	void RawVideo()
	{
		sensor_msgs::ImagePtr out_frame;
		ros::Rate loop_rate(25); // run smoother (no processing delay necessary)
		while (nh_.ok())
		{
			if (!current_grayframe.empty())
			{
				cv::imshow ("Raw Video Feed", current);
				cv::waitKey(1);
				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", current).toImageMsg();
				// output modified video stream
				image_pub_.publish(out_frame);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
