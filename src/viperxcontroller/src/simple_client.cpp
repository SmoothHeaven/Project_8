#include "viperxcontroller/srv/move_to_position.hpp"
#include "rclcpp/rclcpp.hpp"

class MoveToPositionClient : public rclcpp::Node
{
public:
  MoveToPositionClient()
  : Node("move_to_position_client")
  {
    client_ = this->create_client<viperxcontroller::srv::MoveToPosition>("move_to_position");
  }
  //Function to send a goal to the server
  void send_goal(const std::string& positions, const std::string& orientations)
  {
    auto request = std::make_shared<viperxcontroller::srv::MoveToPosition::Request>();
    request->positions = positions;
    request->orientations = orientations;

    auto result_future = client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Goal was accepted by the server: %s", result_future.get()->message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service move_to_position");
    }
  }

private:
  rclcpp::Client<viperxcontroller::srv::MoveToPosition>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MoveToPositionClient>();

  //test values for the client
  std::string positions = "0.0,0.2,0.3;0.3,0.2,0.2";
  std::string orientations = "0.0,0.0,0.0,1.0;0.0,0.0,0.0,1.0";

  action_client->send_goal(positions, orientations);

  rclcpp::shutdown();
  return 0;
}