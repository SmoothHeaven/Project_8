#include "viperxcontroller/viperx_controller.hpp"
#include <vector>
#include <string>
#include <sstream>

namespace viperx {

  ViperXController::ViperXController(std::shared_ptr<rclcpp::Node> node, const std::string& planning_group)
    : node_(node), move_group(node_, planning_group) {
      // Create a service that will move the robot to a specified position and orientation
      service_ = node_->create_service<viperxcontroller::srv::MoveToPosition>("move_to_position", 
        [this](const std::shared_ptr<viperxcontroller::srv::MoveToPosition::Request> request,
              std::shared_ptr<viperxcontroller::srv::MoveToPosition::Response> response) {
              // Parse the positions and orientations from the request
              std::vector<std::string> positions = splitString(request->positions, ';');
              std::vector<std::string> orientations = splitString(request->orientations, ';');

              // Call the moveToPositions function
              bool success = moveToPositions(positions, orientations);

              // Set the response
              response->success = success;
              response->message = success ? "Successfully moved to position" : "Failed to move to position";
        });
  }

  // Function to split a string into a vector of strings, using a specified delimiter
  std::vector<std::string> ViperXController::splitString(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);limiter)) {
      tokens.push_back(token);
    }
    return tokens; 
  }

  // Function to split a string into a vector of doubles, using a specified delimiter
  std::vector<double> split(const std::string &s, char delimiter) {
    std::vector<double> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
      tokens.push_back(std::stod(token));
    }
    return tokens;
  }
  
  // Function to move the robot to a series of positions and orientations
  bool ViperXController::moveToPositions(const std::vector<std::string>& positions, const std::vector<std::string>& orientations) {
      for (std::size_t i = 0; i < positions.size(); i++) {
        geometry_msgs::msg::Pose target_pose;


        // Split the position and orientation strings into their respective components
        std::vector<double> position = split(positions[i], ',');
        std::vector<double> orientation = split(orientations[i], ',');


        // Set the target pose's position and orientation
        target_pose.position.x = position[0];
        target_pose.position.y = position[1];
        target_pose.position.z = position[2];
        target_pose.orientation.x = orientation[0];
        target_pose.orientation.y = orientation[1];
        target_pose.orientation.z = orientation[2];
        target_pose.orientation.w = orientation[3];

        move_group.setPoseTarget(target_pose);

        // Create a plan to that target pose
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = static_cast<bool>(move_group.plan(my_plan));

        // Execute the plan
        if (success) {
          move_group.execute(my_plan); // Correctly pass the plan to execute
        } else {
          RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
          return false;
        }
      }
      return true; // Return true only if all plans have been successfully executed
  }
  
} //namespace viperx