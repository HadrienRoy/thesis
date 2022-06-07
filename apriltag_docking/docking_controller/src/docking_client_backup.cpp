#include "rclcpp/rclcpp.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/srv/docking.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("docking_client");

    auto client = node->create_client<docking_interfaces::srv::Docking>("docking_controller/docking_service");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_WARN(node->get_logger(),"Docking_client was interrupted while waiting for the service. Exiting.\n");
            return 0;
        }
        RCLCPP_WARN(node->get_logger(), "Waiting for the docking server to be up...");
    }

    std::string docking_request = node->declare_parameter("docking_request", "start");

    auto request = std::make_shared<docking_interfaces::srv::Docking::Request>();
    request->service = docking_request;


    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        if (future.get()->success)
        {
            RCLCPP_INFO(node->get_logger(), "Docking service request (docking:=%s) successful.", docking_request.c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Docking service request (docking:=%s) failed.", docking_request.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Error while calling service");
    }

    rclcpp::shutdown();
    return 0;
}






#include "rclcpp/rclcpp.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/srv/docking.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

using std::placeholders::_1;

class DockingClient : public rclcpp::Node
{
    public:
        DockingClient() : Node("docking_client")
        {
            std::thread(std::bind(&DockingClient::callDockingService, this));

            battery_subscriber = this->create_subscription<sensor_msgs::msg::BatteryState>(
                "gazebo_battery_state/battery_state", 10, std::bind(&DockingClient::callbackBattery, this, _1)
            );
        }

        void callDockingService()
        {
            auto client = this->create_client<docking_interfaces::srv::Docking>("docking_controller/docking_service");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }

            std::string docking_request = this->declare_parameter("docking_request", "start");

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
        std::thread thread1;

        sensor_msgs::msg::BatteryState last_battery_state;
        float battery_charge;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_subscriber;

        void callbackBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
        {
            last_battery_state.charge = msg->charge;
            battery_charge = last_battery_state.charge;
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