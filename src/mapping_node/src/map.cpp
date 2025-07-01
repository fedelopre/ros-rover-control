#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <random>

class MappingNode : public rclcpp::Node {
  public:
  MappingNode() : Node("mapping_node"), gen_(rd_()), dist_(0,1), count(0) {
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/vel_mapping", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MappingNode::controlLoop, this));
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count;
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_int_distribution<> dist_;

    void controlLoop() {
      auto cmd = geometry_msgs::msg::Twist(); 
      int random_bit = dist(gen); 
      if (count == 10) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.5;  // Gira a sinistra
        count = 0;
      } 
      else {
          cmd.linear.x = 0.3;
          cmd.angular.z = 0.0;
      }
      if(random_bit){
        cmd.angular.z *= -1;
      }
      count++;
      cmd_pub_->publish(cmd);
    }
  }

  int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}
