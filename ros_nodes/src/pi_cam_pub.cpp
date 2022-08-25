#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_pub_node");
    ros::NodeHandle n;

    cv::VideoCapture cap(-1);
    
    if(!cap.isOpened())
    {
        std::cerr << "Cannot open camera" << std::endl;
        return 1;
    }

    //image_transport::ImageTransport it(n);
    //image_transport::Publisher camPub = it.advertise("/image", 1);
    ros::Publisher camPub = n.advertise<sensor_msgs::Image>("/image", 1);

    ros::Rate loop_rate(10);

    cv::Mat temp0;
    while(ros::ok())
    {

        cap >> temp0;
        if(temp0.empty())
        {
            ROS_INFO("end");
            continue;
        }
        cv::resize(temp0,temp0,Size(640,480),3);
        cv::cvtColor(temp0,temp0,COLOR_BGR2GRAY);
        cv::flip(temp0,temp0, -1);
        cv_bridge::CvImage cvi;
        sensor_msgs::Image im;
        cvi.header.frame_id = "image";
        cvi.header.stamp = ros::Time::now();
        cvi.encoding = "mono8";

	cvi.image=temp0;

        cvi.toImageMsg(im);
        camPub.publish(im);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
