/*
 * image_listener.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: darrenl
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string TOPIC_NAME = "/zed2/zed_node/right/image_rect_color";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
//        Rect bounds(0,0,cv_ptr.cols,cv_ptr.rows);
//        Rect r(200,300,500,500); // partly outside
//        Mat roi = cv_ptr( r & bounds ); // cropped to fit image
//        cv::imwrite("rgb.bmp", cv_ptr->image);
//        cv::imshow("view", cv_bridge::toCvShare(roi, "bgr8")->image);
//        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    //cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1,
            imageCallback);
    ros::spin();
    //cv::destroyWindow("view");
    ros::shutdown();
    return 0;
}
