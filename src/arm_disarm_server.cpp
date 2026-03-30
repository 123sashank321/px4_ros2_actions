#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "px4_ros2_actions_cpp/action/arm.hpp"
#include "px4_ros2_actions_cpp/action/disarm.hpp"

using namespace std::chrono_literals;

class ArmDisarmServer : public rclcpp::Node
{
public:
  using Arm = px4_ros2_actions_cpp::action::Arm;
  using Disarm = px4_ros2_actions_cpp::action::Disarm;
  using GoalHandleArm = rclcpp_action::ServerGoalHandle<Arm>;
  using GoalHandleDisarm = rclcpp_action::ServerGoalHandle<Disarm>;

  explicit ArmDisarmServer() : Node("arm_disarm_server"), current_arming_state_(0), latest_ack_command_(0), ack_result_(0), ack_received_(false)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

    vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      std::bind(&ArmDisarmServer::ack_callback, this, std::placeholders::_1));

    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", qos,
      std::bind(&ArmDisarmServer::status_callback, this, std::placeholders::_1));

    arm_server_ = rclcpp_action::create_server<Arm>(
      this, "arm",
      std::bind(&ArmDisarmServer::handle_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmDisarmServer::handle_arm_cancel, this, std::placeholders::_1),
      std::bind(&ArmDisarmServer::handle_arm_accepted, this, std::placeholders::_1));

    disarm_server_ = rclcpp_action::create_server<Disarm>(
      this, "disarm",
      std::bind(&ArmDisarmServer::handle_disarm_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmDisarmServer::handle_disarm_cancel, this, std::placeholders::_1),
      std::bind(&ArmDisarmServer::handle_disarm_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<Arm>::SharedPtr arm_server_;
  rclcpp_action::Server<Disarm>::SharedPtr disarm_server_;
  
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

  uint8_t current_arming_state_;
  uint32_t latest_ack_command_;
  uint8_t ack_result_;
  bool ack_received_;
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;

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
    current_arming_state_ = msg->arming_state;
  }

  void publish_command(uint32_t command, float param1 = 0.0)
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = command;
    msg.param1 = param1;
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

  // ==== Arm Actions ====
  rclcpp_action::GoalResponse handle_arm_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Arm::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received arm request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_arm_cancel(const std::shared_ptr<GoalHandleArm> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_arm_accepted(const std::shared_ptr<GoalHandleArm> goal_handle)
  {
    std::thread{std::bind(&ArmDisarmServer::execute_arm, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_arm(const std::shared_ptr<GoalHandleArm> goal_handle)
  {
    auto result = std::make_shared<Arm::Result>();

    if (current_arming_state_ == 2) { // 2 = ARMED
      RCLCPP_INFO(this->get_logger(), "Vehicle is already ARMED");
      result->outcome = true;
      result->message = "Vehicle is already in Armed state";
      goal_handle->succeed(result);
      return;
    }

    publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    bool success = wait_for_ack(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);

    result->outcome = success;
    result->message = "Arming " + std::string(success ? "Succeeded" : "Failed");

    if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  // ==== Disarm Actions ====
  rclcpp_action::GoalResponse handle_disarm_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Disarm::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received disarm request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_disarm_cancel(const std::shared_ptr<GoalHandleDisarm> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_disarm_accepted(const std::shared_ptr<GoalHandleDisarm> goal_handle)
  {
    std::thread{std::bind(&ArmDisarmServer::execute_disarm, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_disarm(const std::shared_ptr<GoalHandleDisarm> goal_handle)
  {
    auto result = std::make_shared<Disarm::Result>();

    if (current_arming_state_ == 1) { // 1 = DISARMED
      RCLCPP_INFO(this->get_logger(), "Vehicle is already DISARMED");
      result->outcome = true;
      result->message = "Vehicle is already in Disarmed state";
      goal_handle->succeed(result);
      return;
    }

    publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    bool success = wait_for_ack(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);

    result->outcome = success;
    result->message = "Disarming " + std::string(success ? "Succeeded" : "Failed");

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
  auto node = std::make_shared<ArmDisarmServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
