#ifndef DELIVERY_ROBOT_CORE__DELIVERY_MANAGER_HPP_
#define DELIVERY_ROBOT_CORE__DELIVERY_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <queue>
#include <string>
#include <vector>
#include <map>
#include <memory>

namespace delivery_robot_core
{

struct DeliveryTask
{
    std::string task_id;
    std::string pickup_location;
    std::string dropoff_location;
    geometry_msgs::msg::PoseStamped pickup_pose;
    geometry_msgs::msg::PoseStamped dropoff_pose;
    std::string package_id;
    int priority;  // Higher number = higher priority
    double timeout;  // Task timeout in seconds
    rclcpp::Time creation_time;
};

enum class RobotState
{
    IDLE,
    MOVING_TO_PICKUP,
    WAITING_AT_PICKUP,
    MOVING_TO_DROPOFF,
    WAITING_AT_DROPOFF,
    RETURNING_TO_BASE,
    ERROR,
    CHARGING
};

class DeliveryManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    DeliveryManager();
    ~DeliveryManager();

    // Public API
    bool addDeliveryTask(const DeliveryTask& task);
    bool cancelCurrentTask();
    std::vector<DeliveryTask> getPendingTasks() const;
    RobotState getCurrentState() const { return current_state_; }
    std::string getStateString() const;

private:
    // Callbacks
    void timerCallback();
    void statusPublisher();
    
    // Navigation action callbacks
    void navigateToPoseResponseCallback(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle);
    void navigateToPoseFeedbackCallback(
        rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void navigateToPoseResultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result);
    
    // Waypoints action callbacks
    void followWaypointsResponseCallback(rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr goal_handle);
    void followWaypointsFeedbackCallback(
        rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr goal_handle,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void followWaypointsResultCallback(const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult & result);
    
    // Task management
    void processNextTask();
    void completeCurrentTask(bool success, const std::string& message);
    bool navigateToLocation(const geometry_msgs::msg::PoseStamped& pose);
    bool waitAtLocation(double duration);
    void handleError(const std::string& error_message);
    
    // State transitions
    void setState(RobotState new_state);
    void publishState();
    
    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    
    std::priority_queue<DeliveryTask, std::vector<DeliveryTask>, 
        std::function<bool(const DeliveryTask&, const DeliveryTask&)>> task_queue_;
    
    std::optional<DeliveryTask> current_task_;
    RobotState current_state_;
    rclcpp::Time state_start_time_;
    
    // Configuration
    double wait_at_pickup_duration_;
    double wait_at_dropoff_duration_;
    double task_timeout_;
    int max_retries_;
    std::string base_location_;
    geometry_msgs::msg::PoseStamped base_pose_;
    
    // Metrics
    int tasks_completed_;
    int tasks_failed_;
    double total_distance_traveled_;
    rclcpp::Time start_time_;
};

}  // namespace delivery_robot_core

#endif  // DELIVERY_ROBOT_CORE__DELIVERY_MANAGER_HPP_
