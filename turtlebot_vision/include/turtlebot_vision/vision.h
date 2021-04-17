#ifndef VISION_H
#define VISION_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.h"
#include "sensor_msgs/msg/image.h"
#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include "opencv4/opencv2/highgui.hpp"

class vision : public rclcpp::Node
{
    public:

        vision();
        
        void topiccb(const sensor_msgs::msg::Image::SharedPtr msg);

        void centroid_calc();

        static cv::Rect roi;

    private:

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; 

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        cv_bridge::CvImagePtr depthptr_;

        cv::Point centroid_;

        cv::Moments m;

};

#endif
