#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp" 
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
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr backhome_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscriber;


public:
    SistemaOperativo() : Node("sistemaoperativo"), sem(1)
    {
        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;
        status = '0';
        cControlLoop = 0;
        
        vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

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
            vel_publisher->publish(obstacle_vel);
            break;
        case MAPPING_PRIORITY:
            vel_publisher->publish(mapping_vel);
            break;
        case BACKHOME_PRIORITY:
            vel_publisher->publish(backhome_vel);
            break;
        case NAVIGATION_PRIORITY:
            vel_publisher->publish(nav_vel);
            break;
        }

        obstacle_arrived = nav_arrived = scan_arrived = backhome_arrived = false;
        
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
