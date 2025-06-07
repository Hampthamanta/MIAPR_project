#ifndef NAV2_RRTSTAR_PLANNER__RRT_STAR_PLANNER_HPP_
#define NAV2_RRTSTAR_PLANNER__RRT_STAR_PLANNER_HPP_

#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

struct Point {
    Point()
        : x{0.0}, y{0.0} {}
    Point(double x, double y)
        : x{x}, y{y} {}
    Point(double x, double y, double cost)
        : x{x}, y{y}, cost{cost} {}

    bool operator<(const Point& other) const {
        if (x != other.x) {
            return x < other.x;
        }

        return y < other.y;
    }

    bool operator!=(const Point& other) const {
        return x != other.x || y != other.y;
    }

    double x;
    double y;
    double cost{};
};

namespace nav2_RRTstar_planner {

class RRTstar : public nav2_core::GlobalPlanner {
public:
    RRTstar() = default;
    ~RRTstar() = default;

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

private:
    // Checks if connection between points is valid
    bool is_valid(Point a, Point b);

    // Returns random point
    Point get_random_point();

    // Returns the closest point to given pos
    Point find_closest(Point pos);

    // Returns new point on the segment connecting pt and closest
    Point new_point(Point pt, Point closest);

    std::vector<float> linspace(float start, float stop, std::size_t num_of_points);

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D* costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    // Path points
    std::map<Point, std::shared_ptr<Point>> parent_;

    double interpolation_resolution_;

    double calc_cost(Point start_pt, Point point);
};

}  // namespace nav2_RRTstar_planner

#endif  // NAV2_RRTSTAR_PLANNER__RRT_STAR_PLANNER_HPP_
