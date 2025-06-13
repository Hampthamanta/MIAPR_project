/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <limits>
#include <map>
#include <random>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

//dodanie struktury Point 
struct Point {
    Point(): x{0.0}, y{0.0} {}
    Point(double x, double y): x{x}, y{y} {}

    //przeciążenie operatora <  porównanie dwóch punktów
    bool operator<(const Point& other) const 
    {
        if (x != other.x) 
        {
            return x < other.x;
        }

        return y < other.y;
    }
    //przeciążenie operatora !=  sprawdzenie czy punkty są różne
    bool operator!=(const Point& other) const 
    {
        return x != other.x || y != other.y;
    }

    double x;
    double y;
};


namespace nav2_straightline_planner
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  std::vector<Point> path;
  Point goal_pt;
  bool is_planned=false;
  StraightLine() = default;
  ~StraightLine() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

  // wizualizacja planera
  std::vector<nav_msgs::msg::Path> planning_steps;
  const std::vector<nav_msgs::msg::Path> & getPlanningSteps() const { return planning_steps; }

  std::vector<visualization_msgs::msg::Marker> planning_markers;
  visualization_msgs::msg::Marker start_marker;
  visualization_msgs::msg::Marker goal_marker;
  const std::vector<visualization_msgs::msg::Marker> & getPlanningMarkers() const { return planning_markers; }

private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;

  // -------------MY FUNCTIONS-------------------
  // <key = punkt, value = parent_of_pt> 
  std::map<Point, Point> parent_goal;
  std::map<Point, Point> parent_start;

  // random point
  Point get_random_point();

  // closest point to pos
  // Point find_closest(Point pos);
  Point find_closest(Point pos, const std::map<Point,Point> parents);

  // new point - between rand_pt and closest_pt (step inside)
  Point new_point(Point pt, Point closest);

  // valid - TRUE/FALSE
  bool check_if_valid(Point a, Point b);

  //linspace z overflow
  std::vector<float> linspace(float start, float stop, std::size_t num_of_points);

  // trajektoria z planera
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // wizualizacja planera
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr steps_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr playback_timer_;
  std::size_t playback_index_{0};
  double playback_delay_{0.5};
  cv::Mat playback_image_;
  void startPlayback();


};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
