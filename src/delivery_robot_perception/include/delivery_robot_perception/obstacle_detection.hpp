#ifndef DELIVERY_ROBOT_PERCEPTION__OBSTACLE_DETECTION_HPP_
#define DELIVERY_ROBOT_PERCEPTION__OBSTACLE_DETECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <queue>        // MAKE SURE THIS LINE IS PRESENT
#include <algorithm>
#include <vector>
#include <string>
#include <memory>
#include <limits>
namespace delivery_robot_perception
{

struct Obstacle
{
    std::vector<geometry_msgs::msg::Point> points;
    geometry_msgs::msg::Point centroid;
    double width;
    double height;
    double distance_to_robot;
};

class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode();
    ~ObstacleDetectionNode() = default;

    // Public accessors
    const std::vector<Obstacle>& getObstacles() const;
    double getNearestObstacleDistance() const;
    bool isCollisionWarning() const;

private:
    // Callbacks
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void timerCallback();

    // Processing methods
    std::vector<geometry_msgs::msg::Point> convertScanToPoints(const sensor_msgs::msg::LaserScan& scan);
    std::vector<std::vector<geometry_msgs::msg::Point>> clusterPoints(const std::vector<geometry_msgs::msg::Point>& points);
    void filterClustersBySize();
    void calculateObstacleProperties();
    void checkForCollisions();
    void publishMarkers();

    // Parameters
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double robot_radius_;
    double safety_margin_;
    double update_rate_;
    std::string frame_id_;

    // Data
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::vector<std::vector<geometry_msgs::msg::Point>> clusters_;
    std::vector<Obstacle> obstacles_;
    double nearest_obstacle_distance_;
    bool collision_warning_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace delivery_robot_perception

#endif  // DELIVERY_ROBOT_PERCEPTION__OBSTACLE_DETECTION_HPP_
