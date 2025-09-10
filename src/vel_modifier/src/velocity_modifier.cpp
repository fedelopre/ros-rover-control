#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelSubscriber : public rclcpp::Node
{
    int direction; // quanto è lontano l'oggetto che sto raggiungendo in quella direzione
    rclcpp::TimerBase::SharedPtr timer_;
public:
    VelSubscriber() : Node("velocity_modifier")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&VelSubscriber::Velocity, this));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&VelSubscriber::callback_scan, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
        direction = 1;
    }

private:

    std::mutex m; // Mutex per direction

    /* 
        Serve per la navigazione normale e prende la velocità e la scala per la direction(distanza)
    */
    void Velocity()
    {    
        /* decidi se spostarlo in un timed Loop oppure tenere la call back della velocità */
        //qua modifica la velocità
        auto cmd = geometry_msgs::msg::Twist();

        std::unique_lock<std::mutex> lock(m);
        cmd.linear.x = 0.3*direction;

        publisher_->publish(cmd);
    }

    /* Prende la distanza forntale e la mappa in un valore tra 0 e 1*/
    void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        size_t mid_index = msg->ranges.size() / 2; //davanti
        float front = msg->ranges[mid_index]; 
        // mappare da msg->range_max a range_min  a 1 a 0
        if (std::isinf(front)){
            front = 1;
        } else{ 
            front = (front - msg->range_min) / (msg->range_max - msg->range_min);
        }
        std::unique_lock<std::mutex> lock(m);
        direction = front;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelSubscriber>());
    rclcpp::shutdown();
    return 0;
}
