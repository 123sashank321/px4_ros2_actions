#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

#include "px4_ros2_actions_cpp/action/go_to_position.hpp"

using namespace std::chrono_literals;

class GoToPositionServer : public rclcpp::Node
{
public:
  using GoToPosition = px4_ros2_actions_cpp::action::GoToPosition;
  using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoToPosition>;

  explicit GoToPositionServer() : Node("goto_position_server"), offboard_setpoint_counter_(0), has_target_(false)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", qos);

    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&GoToPositionServer::position_callback, this, std::placeholders::_1));

    vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      std::bind(&GoToPositionServer::ack_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<GoToPosition>(
      this, "goto",
      std::bind(&GoToPositionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GoToPositionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GoToPositionServer::handle_accepted, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&GoToPositionServer::timer_callback, this));
      
    current_position_[0] = 0.0;
    current_position_[1] = 0.0;
    current_position_[2] = 0.0;
  }

private:
  rclcpp_action::Server<GoToPosition>::SharedPtr action_server_;
  
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;

  float current_position_[3];
  uint32_t latest_ack_command_;
  uint8_t ack_result_;
  bool ack_received_;
  
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;

  uint64_t offboard_setpoint_counter_;
  bool has_target_;
  float target_position_[4]; // x, y, z, yaw

  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_position_[0] = msg->x;
    current_position_[1] = msg->y;
    current_position_[2] = msg->z;
  }

  void ack_callback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    latest_ack_command_ = msg->command;
    ack_result_ = msg->result;
    ack_received_ = true;
    ack_cv_.notify_all();
  }

  void timer_callback()
  {
    if (offboard_setpoint_counter_ == 10) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
      offboard_setpoint_counter_++;
    }

    if (has_target_) {
      publish_offboard_control_mode();
      publish_trajectory_setpoint(target_position_[0], target_position_[1], target_position_[2], target_position_[3]);
      
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    }
  }

  void publish_offboard_control_mode()
  {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
  }

  void publish_trajectory_setpoint(float x, float y, float z, float yaw)
  {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.position = {x, y, z};
    msg.yaw = yaw;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
  }

  void publish_vehicle_command(uint32_t command, float param1 = 0.0, float param2 = 0.0)
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
  }

  // ==== GoTo Actions ====
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPosition::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received goto request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoTo> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoTo> goal_handle)
  {
    std::thread{std::bind(&GoToPositionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoTo> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    
    target_position_[0] = goal->x;
    target_position_[1] = goal->y;
    target_position_[2] = goal->z;
    target_position_[3] = goal->yaw;
    has_target_ = true;

    auto feedback = std::make_shared<GoToPosition::Feedback>();
    auto result = std::make_shared<GoToPosition::Result>();

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        has_target_ = false;
        offboard_setpoint_counter_ = 0;
        result->outcome = false;
        result->message = "Cancelled";
        goal_handle->canceled(result);
        return;
      }

      float dx = goal->x - current_position_[0];
      float dy = goal->y - current_position_[1];
      float dz = goal->z - current_position_[2];
      float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      feedback->distance_remaining = dist;
      feedback->state = "Moving";
      goal_handle->publish_feedback(feedback);

      if (dist < 0.5) {
        break;
      }

      loop_rate.sleep();
    }

    has_target_ = false;
    offboard_setpoint_counter_ = 0;

    result->outcome = true;
    result->message = "Reached target";
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPositionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
