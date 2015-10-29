// Contour Finding
flow_contours = flow_th.clone();
cv::findContours (flow_contours, contours, hierarchy,
	CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point (0, 0));

// Approximate contours to polygons + get bounding rects
cv::vector<cv::vector<cv::Point> > contours_poly (contours.size());
cv::vector<cv::Rect> boundRect;
cv::vector<cv::Rect> boundRectRefined;
int count = 0;
cv::Scalar color = cv::Scalar (255, 0, 0);
for (int i = 0; i< contours.size(); i++)
{
	cv::approxPolyDP (cv::Mat(contours[i]), contours_poly[i], 3, true);
	if ((contourArea(contours_poly[i]) < 500) || (contours_poly[i].size() < 20))
	{
		//ROS_INFO("Skipped for area: %f size: ", contourArea(contours[i]));
		continue;
	}
	boundRect.push_back (cv::boundingRect (cv::Mat(contours_poly[i])));
	count++;
}	

// make sure that no two bounding rectangles overlap, so in case they
// do, merge both of them into a single rectangle that contains both

int flag[boundRect.size()];
cv::Rect intersect, rectElement;
for (int i=0; i<boundRect.size(); i++)
{
	if (flag[i] == 1)
	{
		continue;
	}
	rectElement = boundRect[i];
	for (int j = i+1; j< boundRect.size(); j++)
	{
		intersect = (boundRect[i] & boundRect[j]);
		if (intersect.area() > 0)
		{
			flag[j] = 1;
			rectElement = (rectElement | boundRect[j]);
		}
	}
	boundRectRefined.push_back(rectElement);
}

for (int i = 0; i< boundRectRefined.size(); i++)
{
	cv::rectangle (current_grayframe, boundRectRefined[i].tl(),
		boundRectRefined[i].br(), color, 2, 8, 0);
}
