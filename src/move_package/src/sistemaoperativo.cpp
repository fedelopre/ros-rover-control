#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

#define OBSTACLE_PRIORITY 0
#define MAPPING_PRIORITY 1
#define BACKHOME_PRIORITY 2
#define NAVIGATION_PRIORITY 3

class SistemaOperativo : public rclcpp::Node
{
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr posizione_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr backhome_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;

    
    geometry_msgs::msg::Twist mapping_vel;
    geometry_msgs::msg::Twist obstacle_vel;
    geometry_msgs::msg::Twist backhome_vel;
    bool scan_arrived, backhome_arrived, obstacle_arrived, navigation_arrived; // aggiungere nav
    std::string status;
    std::mutex data_mut;


public:
    SistemaOperativo() : Node("sistemaoperativo"), status('0')
    {
        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;
        
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        backhome_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "backhome_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                backhome_vel = *msg;
                backhome_arrived = true;
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        status_subscriber = this->create_subscription<std_msgs::msg::String>(
            "status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                status = msg->data;
                RCLCPP_INFO(this->get_logger(), "Status aggiornato: '%s'", status.c_str());
            });

        scan_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "mapping_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                mapping_vel = *msg;
                scan_arrived = true;
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        nav_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "nav_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                mapping_vel = *msg;
                nav_arrived = true;
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "obstacle_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(m);
                obstacle_vel = *msg;
                obstacle_arrived = true;
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        timer_ = this->create_wall_timer(
            500ms, std::bind(&SistemaOperativo::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(m);
        // Decisione in base a priorità (da implementare con logica più avanzata eventualmente)
        if(obstacle_vel->linear.x == 0){ 
            priority = OBSTACLE_PRIORITY;
        } else {
            priority = std::stoi(status);
        }
        switch (priority)
        {
        case OBSTACLE_PRIORITY:
            vel_publisher->publish(obstacle_vel);
            break;
        case MAPPING_PRIORITY:

            break;
        case BACKHOME_PRIORITY:

            break;
        case NAVIGATION_PRIORITY:
            break;
        }
        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;

        vel_publisher->publish(latest_twist);

        RCLCPP_INFO(this->get_logger(), "Pubblicato messaggio su /cmd_vel");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SistemaOperativo>());
    rclcpp::shutdown();
    return 0;
}
