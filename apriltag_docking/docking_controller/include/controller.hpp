#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"

// #include "apriltag_msgs/msg/april_tag_detection.hpp"
// #include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include "docking_interfaces/msg/current_state.hpp"
#include "docking_interfaces/msg/start_docking.hpp"
#include "docking_interfaces/srv/docking.hpp"
#include "docking_interfaces/srv/gazebo_charge_battery.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "rcl_interfaces/msg/parameter_event.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class DockingController : public rclcpp::Node 
{
    public:
        DockingController() : Node("docking_controller") 
        {
            /*** Declare Parameters ***/

            /*** Define Publishers & Services ***/
            vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            state_publisher = this->create_publisher<docking_interfaces::msg::CurrentState>("docking_controller/current_state", 10);

            docking_service = this->create_service<docking_interfaces::srv::Docking>(
                "docking_controller/docking_service", std::bind(&DockingController::docking_server, this, _1, _2)
            );

            /*** Define Subscribers ***/
            pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
                "detections", 10, std::bind(&DockingController::callbackPose, this, _1)
            );
            // start_docking_subscriber = this->create_subscription<docking_interfaces::msg::StartDocking>(
            //     "start_docking", 10, std::bind(&DockingController::callbackStartDocking, this, _1)
            // );

            // USED FOR VOID DOCKING_TEST
            // timer = this->create_wall_timer(
            //     std::chrono::seconds(1),
            //     std::bind(&DockingController::docking_test, this)
            // );

            RCLCPP_INFO(this->get_logger(), "Docking Controller has been started.");
        }




        void set_docking_state(std::string new_docking_state);
        void run(); // Run for main program

    private:

        rclcpp::TimerBase::SharedPtr timer;

        /*** Variables ***/
        std::string docking_state = "";
        std::string last_docking_state = "";

        // TO DO: using dummy data for now
        std::string action_state = "";
        std::string last_action_state = "";

        double tag_x;
        double tag_y;
        double tag_yaw;

        bool ready_pose = false;
        bool ready_last_pose = false;

        bool temp_start = false;

        /*** Declare Publishers & Services ***/
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher;
        rclcpp::Publisher<docking_interfaces::msg::CurrentState>::SharedPtr state_publisher;

        rclcpp::Service<docking_interfaces::srv::Docking>::SharedPtr docking_service;


        /*** Declare Subscribers & Service Clients ***/
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber;
        //rclcpp::Subscription<docking_interfaces::msg::StartDocking>::SharedPtr start_docking_subscriber;

        // AprilTag Detections Subscriber
        // rclcpp::Subscription<pupil_apriltags

        /*** Declare Messages ***/
        geometry_msgs::msg::Twist cmd_vel_msg;
        geometry_msgs::msg::Pose last_pose;

        
        /*** Declare Functions ***/

        // Callbacks
        void callbackPose(const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            last_pose.position = msg->position;
            last_pose.orientation = msg->orientation;

            ready_pose = true;
        }

        // void callbackStartDocking(const docking_interfaces::msg::StartDocking::SharedPtr msg)
        // {
        //     if (msg->start_docking_controller)
        //     {
        //         // Start docking
        //         set_docking_state("start");
        //     }
        // }

        // Client for gazebo battery charger
        void gazebo_charge_battery_client();
 
        // Turtlebot3 Movement Functions
        void turtlebot_stop();
        void turtlebot_move(double velocity, double radians);
        void turtlebot_turn(double radians);
        void turtlebot_forward(double velocity);

        // State Functions
        void searching_state_func();
        void approach_state_func();
        void final_approach_state_func();
        void docked_state_func();

        void set_action_state(std::string new_action_state);

    
        void docking_test();


        // Controller Functions
        void get_last_pose();
        void docking_state_manager();
        void action_state_manager();
        void publish_state();

        void docking_server(
            const std::shared_ptr<docking_interfaces::srv::Docking::Request> request,
            const std::shared_ptr<docking_interfaces::srv::Docking::Response> response
        );


};

