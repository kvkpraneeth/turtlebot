#include "turtlebot_vision/vision.h"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>

cv::Rect vision::roi(0,0,639,280);

vision::vision() : Node("vision")
{
    this->sub_ = this->create_subscription<sensor_msgs::msg::Image>("/depth_camera", 100, std::bind(&vision::topiccb, this, std::placeholders::_1));
}

void vision::topiccb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    this->depthptr_ = cv_bridge::toCvCopy(msg, msg->encoding);
    // this->centroid_calc();
    cv::imshow("Image Window", this->depthptr_->image);
    cv::waitKey(3);
}

void vision::centroid_calc()
{
    cv::threshold(this->depthptr_->image(roi), this->depthptr_->image, 127, 255, cv::THRESH_BINARY_INV);
    this->m = cv::moments(this->depthptr_->image, true);
    this->centroid_ = cv::Point(m.m10/m.m00, m.m01/m.m00);
    RCLCPP_INFO(this->get_logger(), "Centroid: %d, %d", this->centroid_.x, this->centroid_.y);
}


