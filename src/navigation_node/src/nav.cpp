#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <mutex>


using std::placeholders::_1;

/*THIS IS THE OBSTACLE ONE*/
class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NavigationNode::scanCallback, this, _1));
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/obstacle_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NavigationNode::controlLoop, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex m;
    
    bool obstacle_detected_ = false;
    bool come_back_home = false;
    bool oriented = false;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int front_index = static_cast<int>((0.0 - msg->angle_min)/msg->angle_increment);
    
    if (front_index < 0 || front_index >= static_cast<int>(msg->ranges.size())) {
        RCLCPP_WARN(this->get_logger(), "Indice frontale fuori range");
        return;
    }

    float front = msg->ranges[front_index];

    if (std::isfinite(front)) {
        RCLCPP_INFO(this->get_logger(), "OSTACOLO A DISTANZA %.2f", front);
    } else {
        RCLCPP_INFO(this->get_logger(), "OSTACOLO NON RILEVATO (distanza infinita)");
    }

    std::lock_guard<std::mutex> lock(m);
    if (std::isfinite(front) && front < 0.8) {
        obstacle_detected_ = true;
    } else {
        obstacle_detected_ = false;
    }
    }



    void controlLoop() {
        auto cmd = geometry_msgs::msg::Twist();
        std::lock_guard<std::mutex> lock(m);
        if (obstacle_detected_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;  // Gira a sinistra
        }
        else {
            cmd.linear.x = 0.3;
            cmd.angular.z = 0.0;
        }
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationNode>());
    rclcpp::shutdown();
    return 0;
}
