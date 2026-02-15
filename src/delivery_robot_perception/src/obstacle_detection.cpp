#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <queue>        // ADD THIS LINE
#include <algorithm>
#include <vector>
#include <string>
#include <memory>
#include <limits>
class ObstacleDetectionNode : public rclcpp::Node
{
public:
    ObstacleDetectionNode() : Node("obstacle_detection_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Declare parameters
        this->declare_parameter("obstacle_distance_threshold", 1.0);
        this->declare_parameter("cluster_tolerance", 0.2);
        this->declare_parameter("min_cluster_size", 3);
        this->declare_parameter("robot_radius", 0.4);
        
        // Get parameters
        obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10), std::bind(&ObstacleDetectionNode::scanCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10), std::bind(&ObstacleDetectionNode::odomCallback, this, std::placeholders::_1));
        
        // Publishers
        obstacle_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_markers", 10);
        
        // Timers
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ObstacleDetectionNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Detection Node initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
    }
    
    void timerCallback()
    {
        if (!last_scan_)
            return;
            
        // Convert laser scan to points in robot frame
        std::vector<geometry_msgs::msg::Point> points = laserScanToPoints(*last_scan_);
        
        // Cluster points
        std::vector<std::vector<geometry_msgs::msg::Point>> clusters = clusterPoints(points);
        
        // Filter clusters
        std::vector<std::vector<geometry_msgs::msg::Point>> filtered_clusters = filterClusters(clusters);
        
        // Detect obstacles from clusters
        std::vector<Obstacle> obstacles = detectObstacles(filtered_clusters);
        
        // Check for imminent collisions
        checkCollisions(obstacles);
        
        // Publish visualization markers
        publishObstacleMarkers(obstacles);
    }
    
    std::vector<geometry_msgs::msg::Point> laserScanToPoints(const sensor_msgs::msg::LaserScan& scan)
    {
        std::vector<geometry_msgs::msg::Point> points;
        
        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            float range = scan.ranges[i];
            
            // Filter invalid ranges
            if (range < scan.range_min || range > scan.range_max || std::isnan(range) || std::isinf(range))
                continue;
                
            // Calculate angle
            float angle = scan.angle_min + i * scan.angle_increment;
            
            // Convert to point
            geometry_msgs::msg::Point point;
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0.0;
            
            points.push_back(point);
        }
        
        return points;
    }
    
    std::vector<std::vector<geometry_msgs::msg::Point>> clusterPoints(
        const std::vector<geometry_msgs::msg::Point>& points)
    {
        std::vector<std::vector<geometry_msgs::msg::Point>> clusters;
        std::vector<bool> visited(points.size(), false);
        
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (visited[i])
                continue;
                
            // Start new cluster
            std::vector<geometry_msgs::msg::Point> cluster;
            std::queue<size_t> queue;
            queue.push(i);
            visited[i] = true;
            
            while (!queue.empty())
            {
                size_t current = queue.front();
                queue.pop();
                cluster.push_back(points[current]);
                
                // Find neighbors
                for (size_t j = 0; j < points.size(); ++j)
                {
                    if (!visited[j] && distance(points[current], points[j]) < cluster_tolerance_)
                    {
                        queue.push(j);
                        visited[j] = true;
                    }
                }
            }
            
            clusters.push_back(cluster);
        }
        
        return clusters;
    }
    
    std::vector<std::vector<geometry_msgs::msg::Point>> filterClusters(
        const std::vector<std::vector<geometry_msgs::msg::Point>>& clusters)
    {
        std::vector<std::vector<geometry_msgs::msg::Point>> filtered;
        
        for (const auto& cluster : clusters)
        {
            if (cluster.size() >= static_cast<size_t>(min_cluster_size_))
            {
                filtered.push_back(cluster);
            }
        }
        
        return filtered;
    }
    
    struct Obstacle
    {
        geometry_msgs::msg::Point centroid;
        double radius;
        double distance;
        double angle;
        bool is_dynamic;
    };
    
    std::vector<Obstacle> detectObstacles(const std::vector<std::vector<geometry_msgs::msg::Point>>& clusters)
    {
        std::vector<Obstacle> obstacles;
        
        for (const auto& cluster : clusters)
        {
            Obstacle obs;
            
            // Calculate centroid
            obs.centroid.x = 0.0;
            obs.centroid.y = 0.0;
            obs.centroid.z = 0.0;
            
            for (const auto& point : cluster)
            {
                obs.centroid.x += point.x;
                obs.centroid.y += point.y;
            }
            
            obs.centroid.x /= cluster.size();
            obs.centroid.y /= cluster.size();
            
            // Calculate radius (max distance from centroid)
            obs.radius = 0.0;
            for (const auto& point : cluster)
            {
                double dist = distance(obs.centroid, point);
                if (dist > obs.radius)
                    obs.radius = dist;
            }
            
            // Calculate distance from robot
            obs.distance = std::sqrt(obs.centroid.x * obs.centroid.x + obs.centroid.y * obs.centroid.y);
            obs.angle = std::atan2(obs.centroid.y, obs.centroid.x);
            
            // Check if obstacle is dynamic (this is simplified - would need tracking over time)
            obs.is_dynamic = false;
            
            obstacles.push_back(obs);
        }
        
        return obstacles;
    }
    
    void checkCollisions(const std::vector<Obstacle>& obstacles)
    {
        for (const auto& obs : obstacles)
        {
            // Check if obstacle is within collision distance
            if (obs.distance - obs.radius < robot_radius_ + 0.1) // Adding small safety margin
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Collision imminent! Obstacle at distance: %.2f m, angle: %.2f rad",
                                    obs.distance, obs.angle);
            }
        }
    }
    
    void publishObstacleMarkers(const std::vector<Obstacle>& obstacles)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        int id = 0;
        for (const auto& obs : obstacles)
        {
            // Sphere marker for obstacle
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position = obs.centroid;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obs.radius * 2;
            marker.scale.y = obs.radius * 2;
            marker.scale.z = obs.radius * 2;
            
            // Color based on distance
            if (obs.distance < 1.0)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else if (obs.distance < 2.0)
            {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            }
            else
            {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 0.5;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            
            marker_array.markers.push_back(marker);
        }
        
        obstacle_markers_pub_->publish(marker_array);
    }
    
    double distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_markers_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    geometry_msgs::msg::Pose current_pose_;
    
    // Parameters
    double obstacle_distance_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    double robot_radius_;
};
