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

#include <cmath>
#include <string>
#include <memory>
#include <cstdlib>
#include <chrono>

#include "visualization_msgs/msg/marker.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

  void StraightLine::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

    plan_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/plan", rclcpp::SystemDefaultsQoS());

    nav2_util::declare_parameter_if_not_declared(node_, name_ + ".playback_delay", rclcpp::ParameterValue(0.5));
    node_->get_parameter(name_ + ".playback_delay", playback_delay_);
    steps_pub_ = node_->create_publisher<nav_msgs::msg::Path>(name_ + "/planning_step", rclcpp::SystemDefaultsQoS());
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(name_ + "/planning_marker", rclcpp::SystemDefaultsQoS());
  }

  void StraightLine::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void StraightLine::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void StraightLine::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  nav_msgs::msg::Path StraightLine::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except start position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    planning_steps.clear();
    planning_markers.clear();

    //----------------------------- RRT - CONNECT ---------------------------------
    auto new_goal_pt = Point{goal.pose.position.x, goal.pose.position.y};

    if(new_goal_pt != goal_pt)
    {
      goal_pt = Point{goal.pose.position.x, goal.pose.position.y};
      is_planned = false;
    }
    

    if (!is_planned)
    {
      // wyciągnięcie punktów: startowy i końcowy
      auto start_pt = Point{start.pose.position.x, start.pose.position.y};
      goal_pt = Point{goal.pose.position.x, goal.pose.position.y};


      // wizualizacja planera
      start_marker.header.frame_id = global_frame_;
      start_marker.header.stamp = node_->now();
      start_marker.ns = name_ + "_markers";
      start_marker.id = 0;
      start_marker.type = visualization_msgs::msg::Marker::SPHERE;
      start_marker.action = visualization_msgs::msg::Marker::ADD;
      start_marker.scale.x = 0.08;
      start_marker.scale.y = 0.08;
      start_marker.scale.z = 0.08;
      start_marker.color.r = 0.0f;
      start_marker.color.g = 1.0f;
      start_marker.color.b = 0.0f;
      start_marker.color.a = 1.0f;
      start_marker.pose.orientation.w = 1.0;
      start_marker.pose.position.x = start_pt.x;
      start_marker.pose.position.y = start_pt.y;

      goal_marker = start_marker;
      goal_marker.id = 1;
      goal_marker.color.r = 1.0f;
      goal_marker.color.g = 0.0f;
      goal_marker.pose.position.x = goal_pt.x;
      goal_marker.pose.position.y = goal_pt.y;


      RCLCPP_INFO(node_->get_logger(), "============INIT PLANNING=============");
      RCLCPP_INFO(node_->get_logger(), "Point start: %f %f", start_pt.x, start_pt.y);
      RCLCPP_INFO(node_->get_logger(), "Point goal: %f %f", goal_pt.x, goal_pt.y);

      // inicjalizacja drzew
      parent_start.clear();
      parent_goal.clear();
      parent_start[start_pt] = Point(10.0,10.0);
      parent_goal[goal_pt] = Point(10.0,10.0);

      int swaps = 2;
      int max_iterations = 100000;
      int i = 0;
      auto new_pt = Point();
      auto closest_in_tree = Point();

      for (i = 0; i < max_iterations; i++)
      {

        auto rand_pt = get_random_point();

        auto closest_pt = find_closest(rand_pt, parent_start);

        new_pt = new_point(rand_pt, closest_pt);

        if (!check_if_valid(closest_pt, new_pt))
        {
          continue; // pomiń punkt, jeżeli nie można połaczyć
          // RCLCPP_INFO(node_->get_logger(), "Point skipped");
        }

        parent_start[new_pt] = closest_pt;

        // wizualizacja planera
        nav_msgs::msg::Path step_path;
        step_path.header.stamp = node_->now();
        step_path.header.frame_id = global_frame_;
        geometry_msgs::msg::PoseStamped p1;
        p1.header = step_path.header;
        p1.pose.position.x = closest_pt.x;
        p1.pose.position.y = closest_pt.y;
        p1.pose.orientation.w = 1.0;
        geometry_msgs::msg::PoseStamped p2 = p1;
        p2.pose.position.x = new_pt.x;
        p2.pose.position.y = new_pt.y;
        step_path.poses.push_back(p1);
        step_path.poses.push_back(p2);
        planning_steps.push_back(step_path);

        visualization_msgs::msg::Marker rand_marker;
        rand_marker.header = step_path.header;
        rand_marker.ns = name_ + "_markers";
        rand_marker.id = planning_markers.size() + 2;
        rand_marker.type = visualization_msgs::msg::Marker::SPHERE;
        rand_marker.action = visualization_msgs::msg::Marker::ADD;
        rand_marker.scale.x = 0.05;
        rand_marker.scale.y = 0.05;
        rand_marker.scale.z = 0.05;
        rand_marker.color.b = 1.0f;
        rand_marker.color.a = 1.0f;
        rand_marker.pose.orientation.w = 1.0;
        rand_marker.pose.position.x = rand_pt.x;
        rand_marker.pose.position.y = rand_pt.y;
        planning_markers.push_back(rand_marker);


        // wyswietlanie ------------------------!!!
        // RCLCPP_INFO(node_->get_logger(), "Closest point: %f %f", closest_pt.x,closest_pt.y);
        // RCLCPP_INFO(node_->get_logger(), "Random point: %f %f", rand_pt.x,rand_pt.y);
        // RCLCPP_INFO(node_->get_logger(), "New point: %f %f", new_pt.x,new_pt.y);
        RCLCPP_INFO(node_->get_logger(), "\n");
        if (swaps % 2 == 0)
        {
          RCLCPP_INFO(node_->get_logger(), "Start tree: start");
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Start tree: goal");
        }
        RCLCPP_INFO(node_->get_logger(), "Tree size: %ld", parent_start.size());
        for (const auto &[child, parent] : parent_start)
        {
          RCLCPP_INFO(node_->get_logger(), "Child: (%f ,%f) Parent: (%f ,%f)", child.x, child.y, parent.x, parent.y);
        }
        RCLCPP_INFO(node_->get_logger(), "\n");
        // wyswietlanie ------------------------!!!

        // szukająca najbliższego wierzchołka z nowego punktu przeciwnego drzewa (czyli tu drzewo: parent_goal)
        closest_in_tree = find_closest(new_pt, parent_goal);
        if (check_if_valid(new_pt, closest_in_tree))
        {

          // wizualizacja planera
          nav_msgs::msg::Path connection;
          connection.header.stamp = node_->now();
          connection.header.frame_id = global_frame_;
          geometry_msgs::msg::PoseStamped cp1;
          cp1.header = connection.header;
          cp1.pose.position.x = new_pt.x;
          cp1.pose.position.y = new_pt.y;
          cp1.pose.orientation.w = 1.0;
          geometry_msgs::msg::PoseStamped cp2 = cp1;
          cp2.pose.position.x = closest_in_tree.x;
          cp2.pose.position.y = closest_in_tree.y;
          connection.poses.push_back(cp1);
          connection.poses.push_back(cp2);
          planning_steps.push_back(connection);

          visualization_msgs::msg::Marker conn_marker;
          conn_marker.header = connection.header;
          conn_marker.ns = name_ + "_markers";
          conn_marker.id = planning_markers.size() + 2;
          conn_marker.type = visualization_msgs::msg::Marker::SPHERE;
          conn_marker.action = visualization_msgs::msg::Marker::ADD;
          conn_marker.scale.x = 0.05;
          conn_marker.scale.y = 0.05;
          conn_marker.scale.z = 0.05;
          conn_marker.color.r = 1.0f;
          conn_marker.color.g = 1.0f;
          conn_marker.color.b = 0.0f;
          conn_marker.color.a = 1.0f;
          conn_marker.pose.orientation.w = 1.0;
          conn_marker.pose.position.x = new_pt.x;
          conn_marker.pose.position.y = new_pt.y;
          planning_markers.push_back(conn_marker);


          RCLCPP_INFO(node_->get_logger(), "\n===========PLANNING ENDED SUCCESFULLY===========\n");
          if (swaps % 2 == 1)
          {
            std::swap(new_pt, closest_in_tree);
            std::swap(parent_start, parent_goal);
          }

          break;
        }

        // podmień drzewa
        std::swap(new_pt, closest_in_tree);
        std::swap(parent_start, parent_goal);
        swaps++;
      }

      //----------------------------- END - RRT - CONNECT ---------------------------------
      std::vector<Point> path_from_start;
      std::vector<Point> path_from_end;
      Point current = Point();

      path_from_start.clear();
      path_from_end.clear();

      RCLCPP_INFO(node_->get_logger(), "Point start: %f %f", start_pt.x, start_pt.y);
      RCLCPP_INFO(node_->get_logger(), "Point goal: %f %f", goal_pt.x, goal_pt.y);
      RCLCPP_INFO(node_->get_logger(), "Connection Point in Start Tree: (%f ,%f)", new_pt.x, new_pt.y);
      RCLCPP_INFO(node_->get_logger(), "Connection Point in End Tree: (%f ,%f)", closest_in_tree.x, closest_in_tree.y);

      // // ścieżka od closest do goal
      current = closest_in_tree;
      path_from_end.push_back(current);
      while (current != goal_pt)
      {
        RCLCPP_INFO(node_->get_logger(), "Child: (%f ,%f)", current.x, current.y);
        current = parent_goal[current];
        RCLCPP_INFO(node_->get_logger(), "Parent: (%f ,%f)", current.x, current.y);
        if (current.x == 10.0 && current.y == 10.0)
        {
          RCLCPP_INFO(node_->get_logger(), "Skipped 10,10");
          break; // pomiń sztucznego rodzica
        }
          if (parent_goal.find(current) == parent_goal.end())
        {
          RCLCPP_ERROR(node_->get_logger(), "Brak rodzica dla punktu (%f, %f)", current.x, current.y);
          break;
        }

        RCLCPP_INFO(node_->get_logger(), "Dodano punkt do ścieżki: (%f, %f)", current.x, current.y);
        path_from_end.push_back(current);
      }
      RCLCPP_INFO(node_->get_logger(), "Zrekonstruowana ścieżka do punktu goal");
      for (const auto &pt : path_from_end)
      {
        RCLCPP_INFO(node_->get_logger(), "Point: (%f, %f)", pt.x, pt.y);
      }



      // ścieżka od new_pt do start (wymaga odwrócenia)
      current = new_pt;
      path_from_start.push_back(current);
      while (current != start_pt)
      {
        current = parent_start[current];
        if (current.x == 10.0 && current.y == 10.0)
          break; // pomiń sztucznego rodzica
        if (parent_start.find(current) == parent_start.end())
        {
          // RCLCPP_ERROR(node_->get_logger(), "Brak rodzica dla punktu (%f, %f)", current.x, current.y);
          break;
        }

        // RCLCPP_INFO(node_->get_logger(), "Dodano punkt do ścieżki: (%f, %f)", current.x, current.y);
        path_from_start.push_back(current);
      }
      std::reverse(path_from_start.begin(), path_from_start.end());
      
      RCLCPP_INFO(node_->get_logger(), "Zrekonstruowana ścieżka do punktu start");
      for (const auto &pt : path_from_start)
      {
        RCLCPP_INFO(node_->get_logger(), "Point: (%f, %f)", pt.x, pt.y);
      }


      // // łaczenie ścieżek
      path = path_from_start;
      path.insert(path.end(), path_from_end.begin(), path_from_end.end());
      if (!path.empty())
      {
        // path.erase(path.begin());
        path.pop_back();
      }
      RCLCPP_INFO(node_->get_logger(), "Zrekonstruowana cała ścieżka:");
      for (const auto &pt : path)
      {
        RCLCPP_INFO(node_->get_logger(), "Point: (%f, %f)", pt.x, pt.y);
      }

// dodanie goal do ścieżki
      path.push_back(goal_pt);
      is_planned = true;
    }
    // INTERPOLCJA ŚCIEŻKI (zagęszczenie)
    //---------------------------------------------------
    double step_size = 0.05; // co ile metrów dodać punkt

    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
      Point p1 = path[i];
      Point p2 = path[i + 1];

      double dx = p2.x - p1.x;
      double dy = p2.y - p1.y;
      double dist = std::hypot(dx, dy);
      int steps = std::max(1, static_cast<int>(dist / step_size));

      for (int j = 0; j <= steps; ++j)
      {
        double t = static_cast<double>(j) / steps;
        double x = p1.x + t * dx;
        double y = p1.y + t * dy;

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
      }


    }
    //---------------------------------------------------

    // // dodawanie punktów z listy path do global path
    // for (auto &point : path)
    // {
    //   geometry_msgs::msg::PoseStamped pose;
    //   pose.pose.position.x = point.x;
    //   pose.pose.position.y = point.y;
    //   pose.pose.position.z = 0.0;
    //   pose.pose.orientation.x = 0.0;
    //   pose.pose.orientation.y = 0.0;
    //   pose.pose.orientation.z = 0.0;
    //   pose.pose.orientation.w = 1.0;
    //   pose.header.stamp = node_->now();
    //   pose.header.frame_id = global_frame_;
    //   global_path.poses.push_back(pose);
    // }

    // dodawanie końcowej pozycji
    // geometry_msgs::msg::PoseStamped goal_pose = goal;
    // goal_pose.header.stamp = node_->now();
    // goal_pose.header.frame_id = global_frame_;
    // global_path.poses.push_back(goal_pose);

    
    plan_publisher_->publish(global_path);
    startPlayback();

    return global_path;
  }
  //---------FUNKCJE---------

  Point StraightLine::get_random_point()
  {
    float x = (((std::rand() / (float)RAND_MAX) * 2) - 1) * costmap_->getSizeInMetersX() / 7.f;
    float y = (((std::rand() / (float)RAND_MAX) * 2) - 1) * costmap_->getSizeInMetersY() / 7.f;

    // RCLCPP_INFO(
    //   node_->get_logger(), "COSTMAP size: %f %f",
    //   costmap_->getSizeInMetersX()/ 7f,costmap_->getSizeInMetersY()/ 7f);
    // +-2.739285714 i +-2.739285714
    return {x, y};
  }

  Point StraightLine::find_closest(Point pos, const std::map<Point, Point> parents)
  {
    double min_dist = 1000000.0;
    Point closest;

    for (auto &[key, value] : parents)
    {
      // pomiń parenta sztucznego
      if (key.x == 0.0 && key.y == 0.0)
      {
        continue;
      }

      double dist = std::hypot(pos.x - key.x, pos.y - key.y);

      if (dist < min_dist)
      {
        min_dist = dist;
        closest = key;
      }
    }

    return closest;
  }

  Point StraightLine::new_point(Point pt, Point closest)
  {
    double step = 0.1;
    double norm = std::hypot(pt.x - closest.x, pt.y - closest.y);
    double new_x = ((pt.x - closest.x) / norm) * step;
    double new_y = ((pt.y - closest.y) / norm) * step;

    Point point = Point(closest.x + new_x, closest.y + new_y);

    return point;
  }

  bool StraightLine::check_if_valid(Point a, Point b)
  {
    auto x_points = linspace(a.x, b.x, 100);
    auto y_points = linspace(a.y, b.y, 100);

    std::vector<Point> points{};

    for (int i = 0; i < x_points.size(); i++)
    {
      points.emplace_back(x_points[i], y_points[i]);
    }

    for (const auto &point : points)
    {
      unsigned mx{};
      unsigned my{};

      // jest poza granicami mapy
      if (!costmap_->worldToMap(point.x, point.y, mx, my))
      {
        return false;
      }

      // jest przeszkoda
      //  RCLCPP_INFO(node_->get_logger(), "Cost: %d", costmap_->getCost(mx, my));
      if (costmap_->getCost(mx, my) >= 230) // 251
      {
        return false;
      }
    }

    return true;
  }
  // gotowa funkcja linspace (w c++ nie ma) - https://stackoverflow.com/questions/27028226/python-linspace-in-c
  std::vector<float> StraightLine::linspace(float start, float stop, std::size_t num_of_points)
  {
    std::vector<float> linspaced{};
    linspaced.reserve(num_of_points);

    if (num_of_points == 0)
    {
      return linspaced;
    }
    else if (num_of_points == 1)
    {
      linspaced.push_back(start);
      return linspaced;
    }

    auto delta = (stop - start) / (num_of_points - 1);

    for (int i = 0; i < num_of_points - 1; ++i)
    {
      linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(stop);

    return linspaced;
  }


  void StraightLine::startPlayback()
  {
    if (!steps_pub_ || !marker_pub_) {
      return;
    }

    if (playback_timer_) {
      playback_timer_->cancel();
    }

    playback_index_ = 0;
    marker_pub_->publish(start_marker);
    marker_pub_->publish(goal_marker);
    auto period = std::chrono::duration<double>(playback_delay_);
    playback_timer_ = node_->create_wall_timer(
      period, [this]() {
        if (playback_index_ < planning_steps.size()) {
          steps_pub_->publish(planning_steps[playback_index_]);
          if (playback_index_ < planning_markers.size()) {
            marker_pub_->publish(planning_markers[playback_index_]);
          }
          playback_index_++;
        } else {
          playback_timer_->cancel();
        }
      });
  }


} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
