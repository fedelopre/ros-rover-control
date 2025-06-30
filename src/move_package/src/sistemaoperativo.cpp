#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SistemaOperativo : public rclcpp::Node
{
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr posizione_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vel_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scan_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr other_publisher;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posizione_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scan_subscriber;

    std::string posizione_risposta = "";
    std::string vel_risposta = "";
    std::string scan_risposta = "";

    public:
        SistemaOperativo() : Node("sistemaoperativo"){
            posizione_publisher = this->create_publisher<std_msgs::msg::String>("position", 10);
            vel_publisher = this->create_publisher<std_msgs::msg::String>("vel",10);
            scan_publisher = this->create_publisher<std_msgs::msg::String>("scan",10);

            posizione_subscriber = this->create_subscription<std_msgs::msg::String>(
            "position_modified", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                posizione_risposta = msg->data;
                RCLCPP_INFO(this->get_logger(), "Ricevuto da position_modified: '%s'", msg->data.c_str());
            });

        vel_subscriber = this->create_subscription<std_msgs::msg::String>(
            "vel_modified", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                vel_risposta = msg->data;
                RCLCPP_INFO(this->get_logger(), "Ricevuto da vel_modified: '%s'", msg->data.c_str());
            });

        scan_subscriber = this->create_subscription<std_msgs::msg::String>(
            "scan_modified", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                scan_risposta = msg->data;
                RCLCPP_INFO(this->get_logger(), "Ricevuto da scan_modified: '%s'", msg->data.c_str());
            });

        
            timer_ = this->create_wall_timer(
                500ms, std::bind(&SistemaOperativo::timer_callback, this));
        }
    private:
        void timer_callback() {
            
            /*Ora guardo tutte le cose che mi sono arrivate e decido quale pubblicare sul topic del robot reale in base alla priorità */

            auto scan_msg = std_msgs::msg::String();
            scan_msg.data = scan_risposta; 
            scan_publisher->publish(scan_msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", scan_msg.data.c_str());
            

            auto message = std_msgs::msg::String();
            message.data = posizione_risposta;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            posizione_publisher->publish(message);
            
            auto vel_msg = std_msgs::msg::String();
            vel_msg.data = vel_risposta;
            vel_publisher->publish(vel_msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", vel_msg.data.c_str());

            
            
            

    }
};



int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SistemaOperativo>());
    rclcpp::shutdown();
    return 0;
}