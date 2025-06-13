#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_straightline_planner/msg/planning_steps.hpp"

using nav2_straightline_planner::msg::PlanningSteps;

class PlaybackNode : public rclcpp::Node
{
public:
  PlaybackNode() : Node("planner_playback")
  {
    step_pub_ = create_publisher<nav_msgs::msg::Path>("planning_step", rclcpp::SystemDefaultsQoS());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("planning_marker", rclcpp::SystemDefaultsQoS());
    declare_parameter<double>("delay", 0.5);

    subscription_ = create_subscription<PlanningSteps>(
      "recorded_steps", rclcpp::SystemDefaultsQoS(),
      std::bind(&PlaybackNode::onSteps, this, std::placeholders::_1));
  }

private:
  void onSteps(const PlanningSteps::SharedPtr msg)
  {
    steps_ = msg->steps;
    markers_ = msg->markers;
    index_ = 0;

    double d = get_parameter("delay").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(d),
      std::bind(&PlaybackNode::onTimer, this));
  }

  void onTimer()
  {
    if (index_ < steps_.size()) {
      step_pub_->publish(steps_[index_]);
      if (index_ < markers_.size()) {
        marker_pub_->publish(markers_[index_]);
      }
      index_++;
    } else {
      timer_->cancel();
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr step_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<PlanningSteps>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<nav_msgs::msg::Path> steps_;
  std::vector<visualization_msgs::msg::Marker> markers_;
  std::size_t index_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>());
  rclcpp::shutdown();
  return 0;
}
