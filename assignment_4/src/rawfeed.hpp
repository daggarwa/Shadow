
	void RawVideo()
	{
		sensor_msgs::ImagePtr out_frame;
		ros::Rate loop_rate(25); // run smoother (no processing delay necessary)
		while (nh1_.ok() and choice == 0)
		{
			if (!current_grayframe.empty())
			{
				out_frame = cv_bridge::CvImage (std_msgs::Header(), "bgr8", current).toImageMsg();
				image_pub_.publish(out_frame);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
