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
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "px4_ros2_actions_cpp/action/takeoff.hpp"
#include "px4_ros2_actions_cpp/action/land.hpp"

using namespace std::chrono_literals;

class TakeoffLandServer : public rclcpp::Node
{
public:
  using Takeoff = px4_ros2_actions_cpp::action::Takeoff;
  using Land = px4_ros2_actions_cpp::action::Land;
  using GoalHandleTakeoff = rclcpp_action::ServerGoalHandle<Takeoff>;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  explicit TakeoffLandServer() : Node("takeoff_land_server"), current_altitude_(0.0), latest_ack_command_(0), ack_result_(0), ack_received_(false), nav_state_(0)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

    vehicle_local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      std::bind(&TakeoffLandServer::position_callback, this, std::placeholders::_1));

    vehicle_command_ack_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", qos,
      std::bind(&TakeoffLandServer::ack_callback, this, std::placeholders::_1));

    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", qos,
      std::bind(&TakeoffLandServer::status_callback, this, std::placeholders::_1));

    takeoff_server_ = rclcpp_action::create_server<Takeoff>(
      this, "takeoff",
      std::bind(&TakeoffLandServer::handle_takeoff_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TakeoffLandServer::handle_takeoff_cancel, this, std::placeholders::_1),
      std::bind(&TakeoffLandServer::handle_takeoff_accepted, this, std::placeholders::_1));

    land_server_ = rclcpp_action::create_server<Land>(
      this, "land",
      std::bind(&TakeoffLandServer::handle_land_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TakeoffLandServer::handle_land_cancel, this, std::placeholders::_1),
      std::bind(&TakeoffLandServer::handle_land_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
  rclcpp_action::Server<Land>::SharedPtr land_server_;
  
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;

  float current_altitude_;
  uint32_t latest_ack_command_;
  uint8_t ack_result_;
  bool ack_received_;
  uint8_t nav_state_;
  
  std::mutex ack_mutex_;
  std::condition_variable ack_cv_;

  void position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_altitude_ = -msg->z; // NED to positive UP
  }

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

  void publish_command(uint32_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0)
  {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
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

  // ==== Takeoff Actions ====
  rclcpp_action::GoalResponse handle_takeoff_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Takeoff::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received takeoff request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_takeoff_cancel(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_takeoff_accepted(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    std::thread{std::bind(&TakeoffLandServer::execute_takeoff, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_takeoff(const std::shared_ptr<GoalHandleTakeoff> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Takeoff::Feedback>();
    auto result = std::make_shared<Takeoff::Result>();

    publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 
                    goal->pitch, 0.0, 0.0, goal->yaw, goal->latitude, goal->longitude, goal->altitude);

    if (!wait_for_ack(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF)) {
      result->outcome = false;
      result->message = "Takeoff command rejected or timed out";
      goal_handle->abort(result);
      return;
    }

    float target_alt = goal->altitude > 0 ? goal->altitude : 2.5;

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->outcome = false;
        result->message = "Cancelled";
        goal_handle->canceled(result);
        return;
      }

      feedback->current_altitude = current_altitude_;
      feedback->state = "Climbing";
      goal_handle->publish_feedback(feedback);

      if (std::abs(current_altitude_ - target_alt) < 0.3) {
        break;
      }

      loop_rate.sleep();
    }

    result->outcome = true;
    result->message = "Takeoff completed";
    goal_handle->succeed(result);
  }

  // ==== Land Actions ====
  rclcpp_action::GoalResponse handle_land_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received land request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_land_cancel(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_land_accepted(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    std::thread{std::bind(&TakeoffLandServer::execute_land, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute_land(const std::shared_ptr<GoalHandleLand> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Land::Feedback>();
    auto result = std::make_shared<Land::Result>();

    publish_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 
                    0.0, 0.0, 0.0, goal->yaw, goal->latitude, goal->longitude, goal->altitude);

    if (!wait_for_ack(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND)) {
      result->outcome = false;
      result->message = "Land command rejected or timed out";
      goal_handle->abort(result);
      return;
    }

    rclcpp::Rate loop_rate(2); // 2 Hz
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->outcome = false;
        result->message = "Cancelled";
        goal_handle->canceled(result);
        return;
      }

      feedback->current_altitude = current_altitude_;
      feedback->state = "Descending";
      goal_handle->publish_feedback(feedback);

      if (current_altitude_ < 0.2) {
        break;
      }

      loop_rate.sleep();
    }

    result->outcome = true;
    result->message = "Landing completed";
    goal_handle->succeed(result);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffLandServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
