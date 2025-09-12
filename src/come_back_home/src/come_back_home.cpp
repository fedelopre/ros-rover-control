#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

#include <mutex>
#include <ctime>

using std::placeholders::_1;

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("come_back_home") {

        /*          PUBLISHER           */
        status_publisher = this->create_publisher<std_msgs::msg::String>("status", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/backhome_vel", 10);

        /*          SUBSCRIBER          */
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationNode::odomCallback, this, _1));
        
        // prendo lo status per vedere quando iniziare
        status_subscriber = this->create_subscription<std_msgs::msg::String>(
            "status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                status = msg->data;
                if (std::stoi(status) == 2 && !come_back_home){ // non ancora iniziato
                    startTime = time(NULL);
                    come_back_home = 1; 
                }
            });
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NavigationNode::controlLoop, this));
    }

private:
    std::string status;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex m; // Mutex per le variabili e per il controlLoop
    time_t startTime;

    bool come_back_home = false;
    bool oriented = false;


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        /* Nessuno per ora lo sveglia, serve pubblicare 2 su status */
        if(come_back_home){
            int reached_home = 0;
            const auto& q_msg = msg->pose.pose.orientation;
            tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
            tf2::Vector3 forward_local(1.0, 0.0, 0.0);
            tf2::Vector3 forward_global = tf2::quatRotate(q, forward_local);
            const auto& p_msg = msg->pose.pose.position;
            tf2::Vector3 home(-p_msg.x, -p_msg.y, 0.0);
            home = home.normalized();
            double dot = forward_global.dot(home);
            double angle = std::acos(std::clamp(dot, -1.0, 1.0));

            std::lock_guard<std::mutex> lock(m);
            if(std::abs(angle) < 0.1){
                oriented = true;
            } else {
                oriented = false;
            }
            reached_home = (std::abs(p_msg.x) < 0.1 && std::abs(p_msg.y) < 0.1);
            if ((time(NULL) - startTime > 60) || reached_home){ // dopo un minuto finisce 
                come_back_home = 0;
                auto message = std_msgs::msg::String();
                message.data = "3";
                status_publisher->publish(message); // Inizio a navigare
            }
        }
    }

    void controlLoop() {
        auto cmd = geometry_msgs::msg::Twist();
        std::lock_guard<std::mutex> lock(m);
        if(!oriented && come_back_home){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
        } 
        else if(!come_back_home){
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else{
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
