#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "px4_ros2_actions_cpp/action/set_mode.hpp"

using namespace std::chrono_literals;

class SetModeServer : public rclcpp::Node
{
public:
  using SetMode = px4_ros2_actions_cpp::action::SetMode;
  using GoalHandleSetMode = rclcpp_action::ServerGoalHandle<SetMode>;

  explicit SetModeServer() : Node("set_mode_server"), latest_ack_command_(0), ack_result_(0), ack_received_(false), nav_state_(0)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

    vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      std::bind(&SetModeServer::ack_callback, this, std::placeholders::_1));

    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      std::bind(&SetModeServer::status_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<SetMode>(
      this, "set_mode",
      std::bind(&SetModeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SetModeServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SetModeServer::handle_accepted, this, std::placeholders::_1));
      
    // Map of main_mode and sub_mode
    mode_map_["MANUAL"]     = {1.0, 0.0};
    mode_map_["ALTITUDE"]   = {2.0, 0.0};
    mode_map_["POSITION"]   = {3.0, 0.0};
    mode_map_["MISSION"]    = {4.0, 4.0};
    mode_map_["HOLD"]       = {4.0, 5.0};
    mode_map_["LOITER"]     = {4.0, 5.0};
    mode_map_["RTL"]        = {4.0, 6.0};
    mode_map_["LAND"]       = {4.0, 7.0};
    mode_map_["ACRO"]       = {5.0, 0.0};
    mode_map_["OFFBOARD"]   = {6.0, 0.0};
    mode_map_["STABILIZED"] = {7.0, 0.0};
  }

private:
  rclcpp_action::Server<SetMode>::SharedPtr action_server_;
  
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

  uint32_t latest_ack_command_;
  uint8_t ack_result_;
  bool ack_received_;
  uint8_t nav_state_;
  
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;

  std::map<std::string, std::pair<float, float>> mode_map_;

  void ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    latest_ack_command_ = msg->command;
    ack_result_ = msg->result;
    ack_received_ = true;
    ack_cv_.notify_all();
  }

  void status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    nav_state_ = msg->nav_state;
  }

  void publish_command(uint32_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0)
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

  bool wait_for_ack(uint32_t expected_command, std::chrono::milliseconds timeout = 5000ms)
  {
    std::unique_lock<std::mutex> lock(ack_mutex_);
    ack_received_ = false;
    
    bool got_ack = ack_cv_.wait_for(lock, timeout, [this, expected_command]() {
      return ack_received_ && latest_ack_command_ == expected_command;
    });

    if (got_ack) {
      return ack_result_ == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
    }
    return false;
  }

  // ==== SetMode Actions ====
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SetMode::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received set_mode request: %s", goal->mode_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleSetMode> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetMode> goal_handle)
  {
    std::thread{std::bind(&SetModeServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetMode> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SetMode::Result>();

    std::string mode_request = goal->mode_name;
    // convert to upper
    std::transform(mode_request.begin(), mode_request.end(), mode_request.begin(), ::toupper);

    if (mode_map_.find(mode_request) == mode_map_.end()) {
      RCLCPP_ERROR(this->get_logger(), "Unsupported mode: %s", mode_request.c_str());
      result->outcome = false;
      result->message = "Unsupported mode: " + mode_request;
      goal_handle->abort(result);
      return;
    }

    float main_mode = mode_map_[mode_request].first;
    float sub_mode = mode_map_[mode_request].second;

    publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, main_mode, sub_mode);

    bool success = wait_for_ack(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE);

    result->outcome = success;
    result->message = "Switch to " + mode_request + " " + (success ? "Accepted" : "Failed");

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetModeServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
