// Standard libraries, included
#include <memory>
#include <vector>
#include <iostream>
// OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// ROS Packages
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"


class ArucoDetection : public rclcpp::Node
{
  public:
    ArucoDetection()
    : Node("aruco_detection")
    {
      // Start node properties 
      image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 1, 
        std::bind(&ArucoDetection::aruco_detection_callback, this, std::placeholders::_1)
      );

      image_with_aruco_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "image_aruco", 1
      );
    }

  private:

    void aruco_detection_callback(const sensor_msgs::msg::Image & image)
    { 

      try
      {
        // Pass the image to OpenCV format with enconding BGR8
        cv_ptr_ = cv_bridge::toCvCopy(image, "bgr8");
        // Aruco tag detection definitions
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        std::vector<int> markerIds;
        // Detect aruco markers in the image
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(cv_ptr_->image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        // Draw detected markers
        cv::aruco::drawDetectedMarkers(cv_ptr_->image, markerCorners, markerIds);
        // Convert the cv_image back to sensor_msg::image format
        auto image_aruco = cv_ptr_->toImageMsg();
        image_with_aruco_publisher_->publish(*image_aruco);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }
    
    // Subscribe to the image info topic
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    
    // Publish Image with the aruco detection
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_with_aruco_publisher_;

    // Will get the OpenCV Image
    cv_bridge::CvImagePtr cv_ptr_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetection>());
  rclcpp::shutdown();
  return 0;
}