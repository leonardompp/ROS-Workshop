#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm> 
#include <vector> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "spaceship_interfaces/srv/status.hpp"

/* 

This file implements a fictional spaceship :-)

Its goal is to offer a practical execise in writing a basic ROS 2 CPP file

Follow along with the instructions file 

*/

class Spaceship : public rclcpp::Node
{
  public:
    Spaceship()
    : Node("voyager_spaceship")
    {
      // Part 4: Parameter manipulation
      //this->declare_parameter("components", std::vector<std::string>{"Computer", "Engines", "Network"}); // Declare parameter

      // Part 1: Distance publishing
      this->distance_publisher_ = this->create_publisher<std_msgs::msg::String>("distance", 10); // Announce the topic that will be published to
      this->distance_publishing_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Spaceship::distance_publishing_callback, this)); // Set a callback timer for a frequency of 1 Hz
      this->start_time_ = this->now(); // Save starting time

      // Part 2: Velocity subscription
      //this->velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, 
      //std::bind(&Spaceship::velocity_subscription_callback, this, std::placeholders::_1)); // Announce the topic that will listen to velocities + callback

      // Part 3: Status feedback service
      //this->status_server_ = this->create_service<spaceship_interfaces::srv::Status>("/get_status", 
      //std::bind(&Spaceship::system_status, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    // Part 1: Distance publishing
    void distance_publishing_callback()
    { 
      // Start the message format
      //auto message = std_msgs::msg::String();
      
      // Math to estimate current distance 
      //long speed_to_sun = 17; // kps
      //long initial_distance_to_sun = 24'579'015'078; // km
      //long current_distance = initial_distance_to_sun + speed_to_sun*(this->seconds_passed_); // Assume roughly one cycle a second
      //this->seconds_passed_++; // Assume roughly one cycle a second

      // Publish the message
      //message.data = "Distance from Sun is " + std::to_string(current_distance) + " km! I am far :-)";
      //this->distance_publisher_->publish(message);
    }

    // Part 2: Velocity subscription
    void velocity_subscription_callback(const geometry_msgs::msg::Twist &msg)
    {
      // Log read speed to console
      //RCLCPP_INFO(this->get_logger(), "Spaceship got the following velocity command: v=(%.2f, %.2f, %.2f), w=(%.2f, %.2f, %.2f)", 
      //msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
      // The implementation of this function will come from dev team :-)
    }

    // Part 3: Status feedback service
    void system_status(const std::shared_ptr<spaceship_interfaces::srv::Status::Request> request,
          std::shared_ptr<spaceship_interfaces::srv::Status::Response> response)
    {
      // Parse requested message
      //std::string component_requested = request->component;
      // Log received component
      //RCLCPP_INFO(this->get_logger(), "Requested status for component %s", component_requested.c_str());
      // Part 4: Read available components from parameters
      //this->components_ = this->get_parameter("components").as_string_array();
      // Check if the requested component is in the list provided and return status
      //int cnt = std::count((this->components_).begin(), (this->components_).end(), component_requested);
      //if(cnt > 0)
      //{
      //  response->status = "Component is good!";
      //}
      //else
      //{
      //  response->status = "Non-existent component!";
      //}

    }

    // Part 1: Create a publisher that says the distance from Sun to the spaceship
    rclcpp::TimerBase::SharedPtr distance_publishing_timer_; // Timer to control publisher frequency
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr distance_publisher_; // Actual publishing mechanism
    rclcpp::Time start_time_; // Save the time when the node starts

    // Part 2: Create a subscriber that listens to velocity commands
    //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;

    // Part 3: Create a service server that yields back the status of components
    //rclcpp::Service<spaceship_interfaces::srv::Status>::SharedPtr status_server_;

    // Part 4: Add parameter manipulation to account for diferent spaceship configurations
    //std::vector<std::string> components_;

    double speed_to_sun_ = 17.0; // kps
    double initial_distance_to_sun_ = 24'579'015'078; // km
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Spaceship>());
  rclcpp::shutdown();
  return 0;
}
