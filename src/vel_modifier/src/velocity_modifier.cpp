#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class VelSubscriber : public rclcpp::Node
{
    int direction; // quanto è lontano l'oggetto che sto raggiungendo in quella direzione
public:
    VelSubscriber() : Node("velocity_modifier")
    {
        subscriber_vel = this->create_subscription<std_msgs::msg::String>(
            "vel", 10,
            std::bind(&VelSubscriber::callback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&VelSubscriber::callback_scan, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("vel_modificata", 10);
        direction = 0.1;
    }

private:

    std::mutex m; // Mutex per direction

    /* 
        Serve per la navigazione normale e prende la velocità e la scala per la direction(distanza)
    */
    void callback(const std_msgs::msg::String::SharedPtr msg)
    {    
        /* decidi se spostarlo in un timed Loop oppure tenere la call back della velocità */
        //qua modifica la velocità
        std_msgs::msg::String nuovo_msg;
        std::lock_guard<std::mutex> lock(m);
        nuovo_msg.data = 0.3*direction;

        RCLCPP_INFO(this->get_logger(), "Pubblico Velocità modificata: '%s'", nuovo_msg.data.c_str());
        publisher_->publish(nuovo_msg);
    }

    /* Prende la distanza forntale e la mappa in un valore tra 0 e 1*/
    void callback_scan(const std_msgs::msg::String::SharedPtr msg)
    {
        size_t mid_index = msg->ranges.size() / 2; //davanti
        float front = msg->ranges[mid_index]; 
        std::lock_guard<std::mutex> lock(m);
        // mappare da msg->range_max a range_min  a 1 a 0
        if (float('inf') == front){
            front = 1;
        } else{ 
            front = (front - msg->range_min) / (msg->range_max - msg->range_min);
        }
        std::lock_guard<std::mutex> lock(m);
        direction = front;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_vel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
