#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "docking_interfaces/srv/gazebo_charge_battery.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class GazeboBatteryState : public rclcpp::Node
{
    public:
        GazeboBatteryState() : Node("gazebo_battery_state")
        {
            /*** Declare Parameters ***/
            // this->declare_parameter<std::string>("cmd_battery", "FULL");
            // this->get_parameter("cmd_battery", cmd_battery);

            /*** Define Publishers & Services ***/
            battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>("gazebo_battery_state/battery_state", 10);

            charge_battery_service = this->create_service<docking_interfaces::srv::GazeboChargeBattery>(
                "gazebo_battery_state/charge_battery_service", std::bind(&GazeboBatteryState::charge_battery_server, this, _1, _2));

            /*** Define Variable ***/
            timer = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&GazeboBatteryState::update_gazebo_battery_state, this));

            // Use Turtlebot3 battery data
            battery_state_msg.voltage = 11.1;
            battery_state_msg.temperature = 20.0;
            battery_state_msg.current = -36.0;
            battery_state_msg.charge = 1.8;
            battery_state_msg.capacity = 1.8;
            battery_state_msg.design_capacity = 1.8;
            battery_state_msg.percentage = 1.0;
            battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_DISCHARGING;
            battery_state_msg.power_supply_health = battery_state_msg.POWER_SUPPLY_HEALTH_GOOD;
            battery_state_msg.power_supply_technology = battery_state_msg.POWER_SUPPLY_TECHNOLOGY_LIPO;
            battery_state_msg.present = true;

            RCLCPP_INFO(this->get_logger(), "Gazebo Battery State Node has been started.");
        }

    private:
        /*** Declare Variables ***/
        rclcpp::TimerBase::SharedPtr timer;
        int counter;
        float battery_level = 1.0;

        std::string cmd_battery;

        /*** Declare Publishers & Services ***/
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
        rclcpp::Service<docking_interfaces::srv::GazeboChargeBattery>::SharedPtr charge_battery_service;

        /*** Declare Messages ***/
        sensor_msgs::msg::BatteryState battery_state_msg;

        /*** Declare Functions ***/
        void update_gazebo_battery_state()
        {
            counter += 1;

            if (counter % 60 == 0)
            {
                counter = 0;

                battery_level -= 0.1;
                battery_state_msg.percentage = battery_level;
            }

            battery_pub->publish(battery_state_msg);
        }

        void charge_battery_server(const docking_interfaces::srv::GazeboChargeBattery::Request::SharedPtr request,
                                const docking_interfaces::srv::GazeboChargeBattery::Response::SharedPtr response)
        {
            if (request->command == "charge")
            {
                RCLCPP_INFO(this->get_logger(), "Charging battery.");

                battery_level = 1.0;
                battery_state_msg.percentage = battery_level;
                battery_state_msg.power_supply_status = battery_state_msg.POWER_SUPPLY_STATUS_FULL;
                battery_pub->publish(battery_state_msg);

                response->success = true;
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboBatteryState>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
