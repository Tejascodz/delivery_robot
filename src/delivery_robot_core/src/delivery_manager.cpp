#include "delivery_robot_perception/obstacle_detection.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <queue>        // JUST ADD THIS ONE LINE
#include <algorithm>
#include <vector>
#include <string>
#include <memory>
#include <limits>

namespace delivery_robot_core
{

DeliveryManager::DeliveryManager()
: Node("delivery_manager"),
  task_queue_([](const DeliveryTask& a, const DeliveryTask& b) {
      return a.priority < b.priority;
  }),
  current_state_(RobotState::IDLE),
  tasks_completed_(0),
  tasks_failed_(0),
  total_distance_traveled_(0.0)
{
    // Declare parameters
    this->declare_parameter("wait_at_pickup_duration", 5.0);
    this->declare_parameter("wait_at_dropoff_duration", 3.0);
    this->declare_parameter("task_timeout", 300.0);
    this->declare_parameter("max_retries", 3);
    this->declare_parameter("base_location", "base");
    this->declare_parameter("base_pose.x", 0.0);
    this->declare_parameter("base_pose.y", 0.0);
    this->declare_parameter("base_pose.z", 0.0);
    this->declare_parameter("base_pose.yaw", 0.0);

    // Get parameters
    wait_at_pickup_duration_ = this->get_parameter("wait_at_pickup_duration").as_double();
    wait_at_dropoff_duration_ = this->get_parameter("wait_at_dropoff_duration").as_double();
    task_timeout_ = this->get_parameter("task_timeout").as_double();
    max_retries_ = this->get_parameter("max_retries").as_int();
    base_location_ = this->get_parameter("base_location").as_string();

    base_pose_.header.frame_id = "map";
    base_pose_.pose.position.x = this->get_parameter("base_pose.x").as_double();
    base_pose_.pose.position.y = this->get_parameter("base_pose.y").as_double();
    base_pose_.pose.position.z = this->get_parameter("base_pose.z").as_double();

    tf2::Quaternion q;
    q.setRPY(0, 0, this->get_parameter("base_pose.yaw").as_double());
    base_pose_.pose.orientation.x = q.x();
    base_pose_.pose.orientation.y = q.y();
    base_pose_.pose.orientation.z = q.z();
    base_pose_.pose.orientation.w = q.w();

    // Action clients
    nav_to_pose_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    follow_waypoints_client_ =
        rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

    // Publishers
    state_pub_ =
        this->create_publisher<std_msgs::msg::String>("robot_state", 10);

    goal_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("current_goal", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DeliveryManager::timerCallback, this));

    // Wait for navigation server
    while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Interrupted while waiting for navigation action server");
            return;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for navigation action server...");
    }

    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Delivery Manager initialized");
}

DeliveryManager::~DeliveryManager()
{
    RCLCPP_INFO(this->get_logger(), "Delivery Manager shutting down");
    RCLCPP_INFO(
        this->get_logger(),
        "Tasks completed: %d, Failed: %d",
        tasks_completed_,
        tasks_failed_);
}

bool DeliveryManager::addDeliveryTask(const DeliveryTask& task)
{
    if (task.pickup_location.empty() || task.dropoff_location.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid task: locations cannot be empty");
        return false;
    }

    DeliveryTask new_task = task;
    new_task.creation_time = this->now();
    task_queue_.push(new_task);

    RCLCPP_INFO(
        this->get_logger(),
        "Added delivery task: %s (Priority: %d)",
        new_task.task_id.c_str(),
        new_task.priority);

    if (current_state_ == RobotState::IDLE) {
        processNextTask();
    }

    return true;
}

bool DeliveryManager::cancelCurrentTask()
{
    if (!current_task_) {
        RCLCPP_WARN(this->get_logger(), "No current task to cancel");
        return false;
    }

    completeCurrentTask(false, "Cancelled by user");
    return true;
}

std::vector<DeliveryTask> DeliveryManager::getPendingTasks() const
{
    std::vector<DeliveryTask> tasks;
    auto temp_queue = task_queue_;

    while (!temp_queue.empty()) {
        tasks.push_back(temp_queue.top());
        temp_queue.pop();
    }

    return tasks;
}

std::string DeliveryManager::getStateString() const
{
    switch (current_state_) {
        case RobotState::IDLE: return "IDLE";
        case RobotState::MOVING_TO_PICKUP: return "MOVING_TO_PICKUP";
        case RobotState::WAITING_AT_PICKUP: return "WAITING_AT_PICKUP";
        case RobotState::MOVING_TO_DROPOFF: return "MOVING_TO_DROPOFF";
        case RobotState::WAITING_AT_DROPOFF: return "WAITING_AT_DROPOFF";
        case RobotState::RETURNING_TO_BASE: return "RETURNING_TO_BASE";
        case RobotState::ERROR: return "ERROR";
        case RobotState::CHARGING: return "CHARGING";
        default: return "UNKNOWN";
    }
}

void DeliveryManager::timerCallback()
{
    if (current_task_) {
        double elapsed =
            (this->now() - current_task_->creation_time).seconds();
        if (elapsed > task_timeout_) {
            handleError("Task timeout");
        }
    }

    publishState();
}

void DeliveryManager::publishState()
{
    std_msgs::msg::String msg;
    msg.data = getStateString();
    state_pub_->publish(msg);
}

void DeliveryManager::processNextTask()
{
    if (task_queue_.empty()) {
        setState(RobotState::IDLE);
        return;
    }

    current_task_ = task_queue_.top();
    task_queue_.pop();

    setState(RobotState::MOVING_TO_PICKUP);
    navigateToLocation(current_task_->pickup_pose);
}

void DeliveryManager::completeCurrentTask(bool success, const std::string& message)
{
    if (!current_task_) return;

    if (success) {
        tasks_completed_++;
    } else {
        tasks_failed_++;
    }

    current_task_.reset();

    if (success && !base_location_.empty()) {
        setState(RobotState::RETURNING_TO_BASE);
        navigateToLocation(base_pose_);
    } else {
        processNextTask();
    }
}

bool DeliveryManager::navigateToLocation(
    const geometry_msgs::msg::PoseStamped& pose)
{
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;

    goal_pub_->publish(pose);

    auto options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
        std::bind(&DeliveryManager::navigateToPoseResponseCallback,
                  this, std::placeholders::_1);

    options.feedback_callback =
        std::bind(&DeliveryManager::navigateToPoseFeedbackCallback,
                  this, std::placeholders::_1, std::placeholders::_2);

    options.result_callback =
        std::bind(&DeliveryManager::navigateToPoseResultCallback,
                  this, std::placeholders::_1);

    nav_to_pose_client_->async_send_goal(goal_msg, options);
    return true;
}

void DeliveryManager::navigateToPoseResponseCallback(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle)
{
    if (!goal_handle) {
        handleError("Navigation goal rejected");
    }
}

void DeliveryManager::navigateToPoseFeedbackCallback(
    rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    static double last_distance = 0.0;
    double current_distance = feedback->distance_remaining;

    if (last_distance > 0) {
        total_distance_traveled_ += (last_distance - current_distance);
    }

    last_distance = current_distance;
}

void DeliveryManager::navigateToPoseResultCallback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        if (current_state_ == RobotState::MOVING_TO_PICKUP) {
            setState(RobotState::WAITING_AT_PICKUP);
            waitAtLocation(wait_at_pickup_duration_);
        } else if (current_state_ == RobotState::MOVING_TO_DROPOFF) {
            setState(RobotState::WAITING_AT_DROPOFF);
            waitAtLocation(wait_at_dropoff_duration_);
        } else if (current_state_ == RobotState::RETURNING_TO_BASE) {
            setState(RobotState::IDLE);
            processNextTask();
        }
    } else {
        handleError("Navigation failed");
    }
}

bool DeliveryManager::waitAtLocation(double duration)
{
    this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(duration * 1000)),
        [this]() {
            if (current_state_ == RobotState::WAITING_AT_PICKUP) {
                setState(RobotState::MOVING_TO_DROPOFF);
                navigateToLocation(current_task_->dropoff_pose);
            } else if (current_state_ == RobotState::WAITING_AT_DROPOFF) {
                completeCurrentTask(true, "Completed");
            }
        });

    return true;
}

void DeliveryManager::handleError(const std::string& error_message)
{
    setState(RobotState::ERROR);

    if (current_task_) {
        completeCurrentTask(false, error_message);
    }
}

void DeliveryManager::setState(RobotState new_state)
{
    if (current_state_ == new_state) return;

    const char* new_state_str = [new_state]() {
        switch (new_state) {
            case RobotState::IDLE: return "IDLE";
            case RobotState::MOVING_TO_PICKUP: return "MOVING_TO_PICKUP";
            case RobotState::WAITING_AT_PICKUP: return "WAITING_AT_PICKUP";
            case RobotState::MOVING_TO_DROPOFF: return "MOVING_TO_DROPOFF";
            case RobotState::WAITING_AT_DROPOFF: return "WAITING_AT_DROPOFF";
            case RobotState::RETURNING_TO_BASE: return "RETURNING_TO_BASE";
            case RobotState::ERROR: return "ERROR";
            case RobotState::CHARGING: return "CHARGING";
            default: return "UNKNOWN";
        }
    }();

    RCLCPP_INFO(
        this->get_logger(),
        "State transition: %s -> %s",
        getStateString().c_str(),
        new_state_str);

    current_state_ = new_state;
    state_start_time_ = this->now();
    publishState();
}

}  // namespace delivery_robot_core

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node =
        std::make_shared<delivery_robot_core::DeliveryManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

