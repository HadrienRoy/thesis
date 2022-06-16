#include "rclcpp/rclcpp.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/srv/docking.hpp"
#include "docking_interfaces/srv/start_april_tag_detection.hpp"

#include "sensor_msgs/msg/battery_state.hpp"


using std::placeholders::_1;


class DockingClient : public rclcpp::Node
{
    public:
        DockingClient() : Node("docking_client")
        {
            /*** SUBSCRIBERS DEFINITIONS***/
            battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "gazebo_battery_state/battery_state", 10, std::bind(&DockingClient::callbackBattery, this, _1)
            );

            RCLCPP_INFO(this->get_logger(), "Docking Client has been started.");
        }

        void callAprilTagDetectionService()
        {
            auto client = this->create_client<docking_interfaces::srv::StartAprilTagDetection>("detect_apriltag_pupil/start_apriltag_detection");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the apriltag detection server to be up...");
            }

            std::string tag_request =  "start";

            auto request = std::make_shared<docking_interfaces::srv::StartAprilTagDetection::Request>();
            request->service = tag_request;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "AprilTag detection service request (docking:=%s) successful.", tag_request.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_INFO(this->get_logger(), "AprilTag detection service request (docking:=%s) failed.", tag_request.c_str());
            }
        }

        void callDockingService()
        {
            auto client = this->create_client<docking_interfaces::srv::Docking>("docking_controller/docking_service");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }

            //std::string docking_request = this->declare_parameter("docking_request", "start");
            std::string docking_request =  "start";

            auto request = std::make_shared<docking_interfaces::srv::Docking::Request>();
            request->service = docking_request;

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Docking service request (docking:=%s) successful.", docking_request.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_INFO(this->get_logger(), "Docking service request (docking:=%s) failed.", docking_request.c_str());
            }
        }

       bool stop_client = false;

    private:
        /*** VARIABLES ***/
        std::thread thread_dock;
        std::thread thread_tag;

        /*** INTERFACES ***/
        sensor_msgs::msg::BatteryState last_battery_state;
        float battery_charge;

        /*** SUBSCRIBERS DECLARATIONS***/
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;

        void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
            // Get last battery message
            last_battery_state.percentage = msg->percentage;
            battery_charge = last_battery_state.percentage;

            // Determine if charge is needed
            // TODO: check battery charge left and needed to get to dock
   
            if (battery_charge < 0.9)
            {
                if (!stop_client)
                {
                    RCLCPP_INFO(this->get_logger(), "Charging Required.");

                    thread_tag = std::thread(std::bind(&DockingClient::callAprilTagDetectionService, this));
                    thread_dock = std::thread(std::bind(&DockingClient::callDockingService, this));
                    
                    thread_tag.detach();
                    thread_dock.detach();

                    stop_client = true;
                }
            }
            
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}