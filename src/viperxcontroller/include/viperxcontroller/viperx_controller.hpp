#pragma once

#include "viperxcontroller/srv/move_to_position.hpp" 

#include <memory> 
#include <string>

#include <eigen3/Eigen/Geometry> 
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

namespace viperx {

    class ViperXController{
        public:
            // Constructor for the ViperXController class
            ViperXController(std::shared_ptr<rclcpp::Node> node, const std::string& planning_group);
            // Function to move the robot to a series of positions and orientations
            bool moveToPositions(const std::vector<std::string>& positions, const std::vector<std::string>& orientations);

        private:
            // ROS2 node
            std::shared_ptr<rclcpp::Node> node_;
            // MoveIt interface for planning and executing motions
            moveit::planning_interface::MoveGroupInterface move_group;
            // Service for moving to a position
            rclcpp::Service<viperxcontroller::srv::MoveToPosition>::SharedPtr service_;
            // Function to split a string into a vector of strings, using a specified delimiter
            std::vector<std::string> splitString(const std::string &s, char delimiter);

        }; //Class:ViperXController
} // Namespace:viperx
