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
      
      // Task 1: Distance publishing
      
      // Task 2: Velocity subscription

      // Task 3: Parameter manipulation

      // Task 4: Status feedback service
    }

  private:
    // Task 1: Distance publishing
    void distance_publishing_callback()
    { 
      // Start the message
      
      // Math to estimate current distance 
      // Calculate the duration from the start of the program

      // Calculate total distance from the Sun

      // Publish the message
    }

    // Task 2: Velocity subscription
    void velocity_subscription_callback(const geometry_msgs::msg::Twist &msg)
    {
      // Log read speed to console
    }

    // Task 4: Status feedback service
    void system_status(const std::shared_ptr<spaceship_interfaces::srv::Status::Request> request,
          std::shared_ptr<spaceship_interfaces::srv::Status::Response> response)
    {
      // Read available components from parameters
      
      // Check if the requested component is in the list provided and return status

    }

    double speed_to_sun_ = 17.0; // kps; collinear; away
    double initial_distance_to_sun_ = 24'579'015'078; // km

    // Task 1: Create a publisher that says the distance from Sun to the spaceship

    // Task 2: Create a subscriber that listens to velocity commands

    // Task 4: Create a service server that yields back the status of components

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Spaceship>());
  rclcpp::shutdown();
  return 0;
}
