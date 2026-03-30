#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "px4_ros2_actions_cpp/action/arm.hpp"
#include "px4_ros2_actions_cpp/action/disarm.hpp"
#include "px4_ros2_actions_cpp/action/takeoff.hpp"
#include "px4_ros2_actions_cpp/action/land.hpp"
#include "px4_ros2_actions_cpp/action/go_to_position.hpp"
#include "px4_ros2_actions_cpp/action/set_mode.hpp"
#include "px4_ros2_actions_cpp/action/execute_mission.hpp"

using namespace std::chrono_literals;

class MissionExecutorServer : public rclcpp::Node
{
public:
  using ExecuteMission = px4_ros2_actions_cpp::action::ExecuteMission;
  using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  explicit MissionExecutorServer() : Node("mission_executor_server")
  {
    action_server_ = rclcpp_action::create_server<ExecuteMission>(
      this, "execute_mission",
      std::bind(&MissionExecutorServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MissionExecutorServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MissionExecutorServer::handle_accepted, this, std::placeholders::_1));

    arm_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::Arm>(this, "arm");
    disarm_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::Disarm>(this, "disarm");
    takeoff_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::Takeoff>(this, "takeoff");
    land_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::Land>(this, "land");
    goto_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::GoToPosition>(this, "goto");
    set_mode_client_ = rclcpp_action::create_client<px4_ros2_actions_cpp::action::SetMode>(this, "set_mode");

    RCLCPP_INFO(this->get_logger(), "Mission Executor initialized");
  }

private:
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;

  rclcpp_action::Client<px4_ros2_actions_cpp::action::Arm>::SharedPtr arm_client_;
  rclcpp_action::Client<px4_ros2_actions_cpp::action::Disarm>::SharedPtr disarm_client_;
  rclcpp_action::Client<px4_ros2_actions_cpp::action::Takeoff>::SharedPtr takeoff_client_;
  rclcpp_action::Client<px4_ros2_actions_cpp::action::Land>::SharedPtr land_client_;
  rclcpp_action::Client<px4_ros2_actions_cpp::action::GoToPosition>::SharedPtr goto_client_;
  rclcpp_action::Client<px4_ros2_actions_cpp::action::SetMode>::SharedPtr set_mode_client_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteMission::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received execute request for mission: %s", goal->mission_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecute> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecute> goal_handle)
  {
    std::thread{std::bind(&MissionExecutorServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleExecute> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteMission::Result>();
    auto feedback = std::make_shared<ExecuteMission::Feedback>();

    std::string mission_input = goal->mission_name;
    std::string mission_file;
    
    // Check if absolute path or local file
    std::filesystem::path path(mission_input);
    if (path.is_absolute()) {
      mission_file = mission_input;
    } else {
      try {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("px4_ros2_actions_cpp");
        std::filesystem::path full_path = std::filesystem::path(pkg_share) / "missions" / mission_input;
        if (full_path.extension() != ".yaml" && full_path.extension() != ".yml") {
          full_path += ".yaml";
        }
        mission_file = full_path.string();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not find package path: %s", e.what());
        result->outcome = false;
        result->message = "Package not found";
        goal_handle->abort(result);
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loading mission from: %s", mission_file.c_str());

    if (!std::filesystem::exists(mission_file)) {
      RCLCPP_ERROR(this->get_logger(), "Mission file not found: %s", mission_file.c_str());
      result->outcome = false;
      result->message = "File not found: " + mission_file;
      goal_handle->abort(result);
      return;
    }

    YAML::Node config;
    try {
      config = YAML::LoadFile(mission_file);
    } catch (const YAML::ParserException& e) {
      RCLCPP_ERROR(this->get_logger(), "Error parsing mission file: %s", e.what());
      result->outcome = false;
      result->message = "Parse error";
      goal_handle->abort(result);
      return;
    }

    if (!config["mission"] || !config["mission"].IsSequence()) {
      RCLCPP_WARN(this->get_logger(), "Mission file is empty or missing 'mission' key");
    }

    auto mission_steps = config["mission"];
    int total_steps = mission_steps.size();
    int completed_steps = 0;

    for (std::size_t i = 0; i < mission_steps.size(); ++i) {
      if (goal_handle->is_canceling()) {
        result->outcome = false;
        result->message = "Mission Cancelled";
        result->steps_completed = completed_steps;
        goal_handle->canceled(result);
        return;
      }

      auto step = mission_steps[i];
      std::string action_name = step["action"].as<std::string>();
      
      feedback->current_step = action_name;
      feedback->progress = static_cast<float>(completed_steps) / total_steps;
      goal_handle->publish_feedback(feedback);

      bool success = run_step(action_name, step["params"]);
      
      if (!success) {
        result->outcome = false;
        result->message = "Failed at step: " + action_name;
        result->steps_completed = completed_steps;
        goal_handle->abort(result);
        return;
      }
      
      completed_steps++;
    }

    result->outcome = true;
    result->message = "Mission Completed Successfully";
    result->steps_completed = completed_steps;
    goal_handle->succeed(result);
  }

  bool run_step(const std::string& action_name, const YAML::Node& params)
  {
    RCLCPP_INFO(this->get_logger(), "Running step: %s", action_name.c_str());
    
    if (action_name == "arm") {
      px4_ros2_actions_cpp::action::Arm::Goal goal;
      return call_action<px4_ros2_actions_cpp::action::Arm>(arm_client_, goal);
    } 
    else if (action_name == "disarm") {
      px4_ros2_actions_cpp::action::Disarm::Goal goal;
      return call_action<px4_ros2_actions_cpp::action::Disarm>(disarm_client_, goal);
    }
    else if (action_name == "takeoff") {
      px4_ros2_actions_cpp::action::Takeoff::Goal goal;
      goal.altitude = params["altitude"] ? params["altitude"].as<float>() : 2.5f;
      goal.latitude = params["latitude"] ? params["latitude"].as<float>() : NAN;
      goal.longitude = params["longitude"] ? params["longitude"].as<float>() : NAN;
      goal.yaw = params["yaw"] ? params["yaw"].as<float>() : NAN;
      goal.pitch = params["pitch"] ? params["pitch"].as<float>() : NAN;
      return call_action<px4_ros2_actions_cpp::action::Takeoff>(takeoff_client_, goal);
    }
    else if (action_name == "land") {
      px4_ros2_actions_cpp::action::Land::Goal goal;
      goal.altitude = params["altitude"] ? params["altitude"].as<float>() : NAN;
      goal.latitude = params["latitude"] ? params["latitude"].as<float>() : NAN;
      goal.longitude = params["longitude"] ? params["longitude"].as<float>() : NAN;
      goal.yaw = params["yaw"] ? params["yaw"].as<float>() : NAN;
      return call_action<px4_ros2_actions_cpp::action::Land>(land_client_, goal);
    }
    else if (action_name == "goto") {
      px4_ros2_actions_cpp::action::GoToPosition::Goal goal;
      goal.x = params["x"] ? params["x"].as<float>() : 0.0f;
      goal.y = params["y"] ? params["y"].as<float>() : 0.0f;
      goal.z = params["z"] ? params["z"].as<float>() : 0.0f;
      goal.yaw = params["yaw"] ? params["yaw"].as<float>() : 0.0f;
      return call_action<px4_ros2_actions_cpp::action::GoToPosition>(goto_client_, goal);
    }
    else if (action_name == "set_mode") {
      px4_ros2_actions_cpp::action::SetMode::Goal goal;
      goal.mode_name = params["mode_name"] ? params["mode_name"].as<std::string>() : "MODE";
      return call_action<px4_ros2_actions_cpp::action::SetMode>(set_mode_client_, goal);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown action: %s", action_name.c_str());
      return false;
    }
  }

  template <typename ActionType>
  bool call_action(typename rclcpp_action::Client<ActionType>::SharedPtr client, const typename ActionType::Goal& goal)
  {
    if (!client->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return false;
    }

    auto send_goal_options = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
    
    auto goal_handle_future = client->async_send_goal(goal, send_goal_options);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call action");
      return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      return false;
    }

    auto result_future = client->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get action result");
      return false;
    }

    auto wrapped_result = result_future.get();
    if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return wrapped_result.result->outcome;
    }
    
    return false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutorServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
