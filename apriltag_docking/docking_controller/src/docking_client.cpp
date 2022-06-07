#include "rclcpp/rclcpp.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/msg/start_docking.hpp"
#include "docking_interfaces/srv/docking.hpp"

#include "sensor_msgs/msg/battery_state.hpp"


using std::placeholders::_1;


class DockingClient : public rclcpp::Node
{
    public:
        DockingClient() : Node("docking_client")
        {
            /*** PARAMETERS ***/

            /*** PUBLISHER DEFINITIONS***/
            start_docking_publisher = this->create_publisher<docking_interfaces::msg::StartDocking>(
                "start_docking", 10
            );

            /*** SUBSCRIBERS DEFINITIONS***/
            battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "gazebo_battery_state/battery_state", 10, std::bind(&DockingClient::callbackBattery, this, _1)
            );


            RCLCPP_INFO(this->get_logger(), "Docking Client has been started.");
        }

        void callDockingService()
        {
            auto client = this->create_client<docking_interfaces::srv::Docking>("docking_controller/docking_service");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }

            std::string docking_request = this->declare_parameter("docking_request", "start");
            //std::string docking_request =  "start";

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

    private:
        /*** VARIABLES ***/
        std::thread thread1;

        /*** INTERFACES ***/
        sensor_msgs::msg::BatteryState last_battery_state;
        float battery_charge;
        docking_interfaces::msg::StartDocking start_docking_msg;

        /*** PUBLISHER DECLARATIONS***/
        rclcpp::Publisher<docking_interfaces::msg::StartDocking>::SharedPtr start_docking_publisher;

        /*** SUBSCRIBERS DECLARATIONS***/
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;

        void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
            // Get last battery message
            last_battery_state.percentage = msg->percentage;
            battery_charge = last_battery_state.percentage;

            // Determine if charge is needed
            // TODO: check battery charge left and needed to get to dock
            if (battery_charge < 1.0)
            {
                start_docking_msg.start_docking_controller = true;
                start_docking_msg.start_apriltag_detection = true;
                
                // thread1 = std::thread(std::bind(&DockingClient::callDockingService, this));
                // thread1.detach();
            }
            start_docking_publisher->publish(start_docking_msg);
            // TODO: check docking states of other robots 
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