#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <memory>
#include <semaphore>

using namespace std::chrono_literals;

#define OBSTACLE_PRIORITY 0
#define MAPPING_PRIORITY 1
#define BACKHOME_PRIORITY 2
#define NAVIGATION_PRIORITY 3

class SistemaOperativo : public rclcpp::Node
{
    double wheel_separation = 0.122;
    double wheel_base = 0.156;
    double wheel_radius = 0.026;
    double wheel_steering_y_offset = 0.03;
    double steering_track = wheel_separation - 2 * wheel_steering_y_offset;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr backhome_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;


public:
    SistemaOperativo() : Node("sistemaoperativo"), sem(1)
    {
        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;
        status = '3';
        cControlLoop = 0;
        
        //vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", 10);

        /*
            I seguenti Subscriber sono implementati con una
            gestione mutex e semafori per la concorrenza con controlLoop
            e altri thread del pacchetto /backhome
        */

        /*
            Subscriber per il ritorno al punto di spawn
        */
        backhome_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "backhome_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mut);
                backhome_vel = *msg;
                backhome_arrived = true;
                if ((obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived) && cControlLoop){
                    sem.release();
                    cControlLoop--;
                }
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });
        
        /*
            Status --> indica la fase della vita del robot
            0: Allerta ostacolo
            1: Sta mappando la zona
            2: Torna al punto di spawn
            3: Navigazione controllata
        */
        status_subscriber = this->create_subscription<std_msgs::msg::String>(
            "status", 10,
            [this](std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mut);
                status = msg->data;
                RCLCPP_INFO(this->get_logger(), "Status aggiornato: '%s'", status.c_str());
            });

        /*
            Riceve le informazioni di navigazione per il Mapping della stanza
        */
        scan_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "mapping_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mut);
                mapping_vel = *msg;
                scan_arrived = true;
                if ((obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived) && cControlLoop){
                    sem.release();
                    cControlLoop--;
                }
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        /*
            Riceve i controlli per il movimento in fase di ricerca
        */
        nav_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "nav_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mut);
                nav_vel = *msg;
                nav_arrived = true;
                if ((obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived) && cControlLoop){
                    sem.release();
                    cControlLoop--;
                }
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        /*
            Riceve l'allerta di ostacoli
            Questa ricezione ha la priorità su tutte le fasi di movimento
        */
        vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "obstacle_vel", 10,
            [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(data_mut);
                obstacle_vel = *msg;
                obstacle_arrived = true;
                //Se sono l'ultimo allora sveglio il controlLoop
                if ((obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived) && cControlLoop){
                    sem.release();
                    cControlLoop--;
                }
                RCLCPP_INFO(this->get_logger(), "Ricevuto Twist linear.x: %.2f", msg->linear.x);
            });

        timer_ = this->create_wall_timer(
            500ms, std::bind(&SistemaOperativo::timer_callback, this));
    }

private:
    geometry_msgs::msg::Twist mapping_vel;
    geometry_msgs::msg::Twist obstacle_vel;
    geometry_msgs::msg::Twist backhome_vel;
    geometry_msgs::msg::Twist nav_vel;
    geometry_msgs::msg::Twist final_vel;
    
    bool scan_arrived, backhome_arrived, obstacle_arrived, nav_arrived; // aggiungere nav
    std::string status;
    std::mutex data_mut;
    std::counting_semaphore<1> sem;
    int cControlLoop;
    void timer_callback()

    {
        std::unique_lock<std::mutex> lock(data_mut);
        int priority;
        if (!(obstacle_arrived && nav_arrived && scan_arrived && backhome_arrived)){
            cControlLoop++;
            lock.unlock();
            sem.acquire();
            lock.lock(); 
        }
        
        if(obstacle_vel.linear.x == 0){ 
            priority = OBSTACLE_PRIORITY;
        } else {
            priority = std::stoi(status);
        }
        switch (priority)
        {
        case OBSTACLE_PRIORITY:
            final_vel = obstacle_vel;
            break;
        case MAPPING_PRIORITY:
            final_vel = mapping_vel;
            break;
        case BACKHOME_PRIORITY:
            final_vel = backhome_vel;
            break;
        case NAVIGATION_PRIORITY:
            final_vel = nav_vel;
            break;
        }

        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;

        double sign = (final_vel.linear.x >= 0.0) ? 1.0 : -1.0;
        double offset = final_vel.angular.z * wheel_steering_y_offset;

        double v0 = sign * hypot(final_vel.linear.x - final_vel.angular.z * steering_track / 2.0,
                                 final_vel.angular.z * wheel_base / 2.0) - offset;
        double v1 = sign * hypot(final_vel.linear.x + final_vel.angular.z * steering_track / 2.0,
                                final_vel.angular.z * wheel_base / 2.0) + offset;
        double v2 = v0;
        double v3 = v1;

        std_msgs::msg::Float64MultiArray vel_msg;
        vel_msg.data = {v0, v1, v2, v3};
        vel_pub_->publish(vel_msg);


        RCLCPP_INFO(this->get_logger(), "Pubblicato messaggio su /forward_velocity/commands");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SistemaOperativo>());
    rclcpp::shutdown();
    return 0;
}
