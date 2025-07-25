#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class VelSubscriber : public rclcpp::Node
{
    int direction; // quanto è lontano l'oggetto che sto raggiungendo in quella direzione
public:
    VelSubscriber() : Node("velocity_modifier")
    {
        subscriber_vel = this->create_subscription<std_msgs::msg::String>(
            "vel", 10,
            std::bind(&VelSubscriber::callback, this, std::placeholders::_1));
        
        subscriber_scan = this->create_subscription<std_msgs::msg::String>(
            "vel", 10,
            std::bind(&VelSubscriber::callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("vel_modificata", 10);
        direction = 0;
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ricevuta Velocità: '%s'", msg->data.c_str());
        
        //qua modifica la velocità
        std_msgs::msg::String nuovo_msg;
        nuovo_msg.data = msg->data*direction;

        RCLCPP_INFO(this->get_logger(), "Pubblico Velocità modificata: '%s'", nuovo_msg.data.c_str());
        publisher_->publish(nuovo_msg);
    }

    void callback_scan(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Ricevuta Velocità: '%s'", msg->data.c_str());
        
        //Salvo la distanza scan
        direction = int(msg->data)/10 //scalalo 

    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_vel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
