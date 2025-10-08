// File: dynamic_pick_sequencer.cpp (Final Corrected Version)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <conveyorbelt_msgs/srv/conveyor_belt_control.hpp>
// #include <rclcpp/parameter_client.hpp> // <-- REMOVED: Not needed

#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <thread>
#include <map> 

using namespace std::chrono_literals;

class DynamicPickSequencer : public rclcpp::Node
{
public:
    DynamicPickSequencer()
        : Node("pipeline_fsm") 
    {
        // 1. Publishers and Clients Setup
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        conveyor_client_ = this->create_client<conveyorbelt_msgs::srv::ConveyorBeltControl>(
            "/conveyor3/CONVEYORPOWER");
        
        // **REMOVED: vision_node_param_client_ initialization**

        // 2. Wait for Services
        RCLCPP_INFO(this->get_logger(), "Waiting for conveyor service...");
        
        // FIX 1: Add 'this->template' keyword to correctly call the template member function
        this->template wait_for_service<conveyorbelt_msgs::srv::ConveyorBeltControl>(conveyor_client_, "/conveyor3/CONVEYORPOWER");
        
        RCLCPP_INFO(this->get_logger(), "Conveyor service ready. Starting pipeline logic.");

        // 3. Start the main execution thread
        main_thread_ = std::thread(&DynamicPickSequencer::run_pipeline, this);
    }

    ~DynamicPickSequencer()
    {
        if (main_thread_.joinable()) {
            main_thread_.join();
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Client<conveyorbelt_msgs::srv::ConveyorBeltControl>::SharedPtr conveyor_client_;
    // **REMOVED: rclcpp::AsyncParametersClient::SharedPtr vision_node_param_client_;**
    std::thread main_thread_;

    // FIX 2: Added the template function definition for wait_for_service
    template<typename T>
    void wait_for_service(typename rclcpp::Client<T>::SharedPtr client, const std::string& service_name)
    {
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service '%s'.", service_name.c_str());
                throw std::runtime_error("ROS shutdown during service wait.");
            }
            RCLCPP_WARN(this->get_logger(), "Service '%s' not available, waiting...", service_name.c_str());
        }
    }

    // **REMOVED: map_power_to_speed function is no longer needed**

    double determine_conveyor_power(double amr_vel_x)
    {
        double speed = std::abs(amr_vel_x);

        if (speed >= 0.10 && speed <= 0.15) {
            return 20.0;
        } else if (speed > 0.15 && speed <= 0.25) {
            return 30.0;
        } else if (speed > 0.25 && speed <= 0.30) {
            return 35.0;
        }
        
        RCLCPP_WARN(this->get_logger(), "AMR speed %.2f m/s is outside the defined range (0.10 to 0.30). Using power 0.0.", amr_vel_x);
        return 0.0;
    }

    void run_pipeline()
    {
        double amr_speed_input = 0.0;
        bool valid_input = false;

        // --- Step A: Get User Input for AMR Speed ---
        while (!valid_input) {
            std::cout << "\n=======================================================\n";
            std::cout << "Enter desired AMR speed (0.1 to 0.3, will be in -X dir): ";
            std::cout << "\n=======================================================\n";
            if (std::cin >> amr_speed_input) {
                if (amr_speed_input >= 0.1 && amr_speed_input <= 0.3) {
                    amr_speed_input = -amr_speed_input;
                    valid_input = true;
                } else {
                    std::cout << "Invalid speed. Please enter a number between 0.1 and 0.3.\n";
                }
            } else {
                std::cout << "Invalid input. Please enter a valid number.\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        
        // --- Step B: Determine Conveyor Power ---
        double conveyor_power = determine_conveyor_power(amr_speed_input);
        
        // --- Step C: Send AMR Speed Command (/cmd_vel) ---
        RCLCPP_INFO(this->get_logger(), "Sending AMR speed command: X=%.2f m/s", amr_speed_input);
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = amr_speed_input;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(twist_msg);
        rclcpp::sleep_for(200ms); 

        // --- Step D: Service Call to Start Conveyor ---
        if (conveyor_power > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Calling service to set Conveyor Power: %.1f", conveyor_power);
            
            auto request = std::make_shared<conveyorbelt_msgs::srv::ConveyorBeltControl::Request>();
            request->power = conveyor_power;

            auto result = conveyor_client_->async_send_request(request);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
                rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "✅ Conveyor started successfully!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "❌ Conveyor failed to start. Service returned success=False."); 
                    return;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Conveyor service call failed (timeout/error).");
                return;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Conveyor power is 0.0. Conveyor not started.");
        }

        // **REMOVED: Step D.5: Parameter update logic that caused compiler errors**

        // --- Step E: Conclude Pipeline ---
        RCLCPP_INFO(this->get_logger(), "Pipeline setup finished. The pick controller should now be active based on box state.");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<DynamicPickSequencer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Pipeline failed to start: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}