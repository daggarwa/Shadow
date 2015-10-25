#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "iostream"
#include "string.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    cv::Mat flow, cflow, frame;
    cv::Mat current_grayframe, prev_grayframe, uflow;
    // cv::UMat gray, prevgray, uflow;
    // namedWindow("flow", 1);
 

public:
    ImageConverter()
      : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
          &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            // current_grayframe = cv_ptr->image;
            cvtColor(cv_ptr->image, current_grayframe, CV_BGR2GRAY);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // // Draw an example circle on the video stream
        // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    }

    // template <typename T>
    // inline T mapVal(T x, T a, T b, T c, T d)
    // {
    //     x = std::max(std::min(x, b), a);
    //     return c + (d-c) * (x-a) / (b-a);
    // }

    // static void colorizeFlow(const cv::Mat &u, const cv::Mat &v, cv::Mat &dst)
    // {
    //     double uMin, uMax;
    //     cv::minMaxLoc(u, &uMin, &uMax, 0, 0);
    //     double vMin, vMax;
    //     cv::minMaxLoc(v, &vMin, &vMax, 0, 0);
    //     uMin = std::abs(uMin); uMax = std::abs(uMax);
    //     vMin = std::abs(vMin); vMax = std::abs(vMax);
    //     float dMax = static_cast<float>(std::max(std::max(uMin, uMax), std::max(vMin, vMax)));
    // 
    //     dst.create(u.size(), CV_8UC3);
    //     for (int y = 0; y < u.rows; ++y)
    //     {
    //         for (int x = 0; x < u.cols; ++x)
    //         {
    //             dst.at<uchar>(y,3*x) = 0;
    //             dst.at<uchar>(y,3*x+1) = (uchar)mapVal(-v.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
    //             dst.at<uchar>(y,3*x+2) = (uchar)mapVal(u.at<float>(y,x), -dMax, dMax, 0.f, 255.f);
    //         }
    //     }
    // }


    // static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
    //                     double, const cv::CvScalar& color)

    static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
                        double)
    {
        for(int y = 0; y < cflowmap.rows; y += step)
            for(int x = 0; x < cflowmap.cols; x += step)
            {
                const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
                cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                     CV_RGB(0, 255, 0));
                cv::circle(cflowmap, cv::Point(x,y), 2, CV_RGB(0, 255, 0), -1);
            }
    }

	std::string type2str(int type) {
		std::string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch ( depth ) {
			case CV_8U:  r = "8U"; break;
			case CV_8S:  r = "8S"; break;
			case CV_16U: r = "16U"; break;
			case CV_16S: r = "16S"; break;
			case CV_32S: r = "32S"; break;
			case CV_32F: r = "32F"; break;
			case CV_64F: r = "64F"; break;
			default:     r = "User"; break;
		}

		r += "C";
		r += (chans+'0');

		return r;
	}

    void OpticalFlow()
    {
 
        double  min, max;

        // cv_bridge::CvImage out_frame;
        sensor_msgs::ImagePtr out_frame;

        cv::Mat flowxy_mag, flowx, flowy, image, prev_rgbframe, flow_th;

        cv::Mat rgb_flow;
    
        ros::Rate loop_rate(5);

        while(nh_.ok())
        {
            if( !current_grayframe.empty() )
            {
                cv::calcOpticalFlowFarneback(prev_grayframe, current_grayframe, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
                // cv::cvtColor(prev_grayframe, cflow, COLOR_GRAY2BGR);
                // uflow.copyTo(flow);
                // drawOptFlowMap(flow, cflow, 16, 1.5, Scalar(0, 255, 0));

                // cv::imshow("flow", flow);
                // if(waitKey(30)>=0)
                //     break;


                // Update GUI Window
                // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
                cv::cvtColor(prev_grayframe, prev_rgbframe, CV_GRAY2BGR);

                //cv::Mat planes[] = {flowx, flowy};
                cv::vector<cv::Mat> planes(2);
                split(flow, planes);
                flowx = planes[0];
                flowy = planes[1];        
                // cv::minMaxLoc(flow, &min, &max);
                magnitude(flowx, flowy, flowxy_mag);
                cv::minMaxLoc(flowxy_mag, &min, &max);
		cv::threshold(flowxy_mag, flow_th, 10, 255, 0);
		
		int dilation_type = cv::MORPH_RECT, dilation_size = 10;
		cv::Mat element = getStructuringElement(dilation_type, cv::Size(10, 10));
		cv::erode(flow_th, flow_th, element);		
		cv::dilate(flow_th, flow_th, element); // confirm same src/dst works

		flow_th.convertTo(flow_th, CV_8U);
		cv::vector<cv::vector<cv::Point> > contours;
		cv::vector<cv::Vec4i> hierarchy;
		//cv::findContours(flow_th, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		cv::findContours(flow_th, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		//cv::Mat drawing = cv::Mat::zeros( prev_grayframe.size(), CV_8UC3 );
		//cv::RNG rng(12345);
		//for( int i = 0; i< contours.size(); i++ )
		//{
		//    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		//    cv::drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
		//}    
		std::string ty = type2str( flow_th.type() );
		ROS_INFO("Matrix: %s %dx%d \n", ty.c_str(), flow_th.cols, flow_th.rows );

                ROS_INFO("Flow Mat: %f %f", min, max);


                // colorizeFlow(flowx, flowy, image);
                
                // drawOptFlowMap(flow, prev_rgbframe, 16, 1.5);
                cv::imshow(OPENCV_WINDOW, flow_th);
                //cv::imshow(OPENCV_WINDOW, flowx);
                cv::waitKey(3);

                // cv_bridge::CvImage out_msg;
                // out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
                // out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
                // out_msg.image    = sal_float_image; // Your cv::Mat
                
                out_frame = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_flow).toImageMsg();

                // Output modified video stream
                image_pub_.publish(out_frame);
            }
            ros::spinOnce();
            loop_rate.sleep();
            std::swap(prev_grayframe, current_grayframe);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ic.OpticalFlow();
    ros::spin();
    return 0;
}
