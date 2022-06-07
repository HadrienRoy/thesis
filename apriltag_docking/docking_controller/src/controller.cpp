#include "controller.hpp"


void DockingController::docking_state_manager()
{
    if (docking_state == "start")
    {
        turtlebot_stop();
        set_docking_state("searching");
    }

    if (docking_state == "searching")
    {
        searching_state_func();
    }

    if (docking_state == "approach")
    {
        approach_state_func();
    }

    if (docking_state == "final_approach")
    {
        final_approach_state_func();
    }

    if (docking_state == "docked")
    {
        docked_state_func();
    }
}

void DockingController::action_state_manager()
{
    if (action_state == "jogging")
    {
        
    }

    if (action_state == "turning")
    {
        
    }

}

void DockingController::docking_test()
{
    run();

    //RCLCPP_INFO(get_logger(),"x position: %f", abs(tag_x));
    if (!ready_last_pose)
    {
        return;
    }

    // TODO: Change position values after camera calibration
    if (abs(tag_x) < 3.0)
    {
        set_docking_state("docked");
    }

    else if (abs(tag_x) < 6.0)
    {
        set_docking_state("final_approach");
    }

    else if (abs(tag_x) > 9.0)
    {
        set_docking_state("approach");
    }

    ready_last_pose = false;
}

/*** Docking State Functions ***/
void DockingController::searching_state_func()
{
    if (!ready_last_pose)
    {
        return;
    }

    // TODO: Change position values after camera calibration
    if (abs(tag_x) < 3.0)
    {
        set_docking_state("docked");
    }

    else if (abs(tag_x) < 6.0)
    {
        set_docking_state("final_approach");
    }

    else if (abs(tag_x) > 9.0)
    {
        set_docking_state("approach");
    }

    ready_last_pose = false;
}

void DockingController::approach_state_func()
{
    turtlebot_forward(0.2);

    if (!ready_last_pose)
    {
        return;
    }

    // TODO: Change position values after camera calibration
    if (abs(tag_x) < 3.0)
    {
        set_docking_state("docked");
    }

    else if (abs(tag_x) < 6.0)
    {
        set_docking_state("final_approach");
    }

    else if (abs(tag_x) > 9.0)
    {
        set_docking_state("approach");
    }

    ready_last_pose = false;
}

void DockingController::final_approach_state_func()
{
    turtlebot_forward(0.1);

    if (!ready_last_pose)
    {
        return;
    }

    // TODO: Change position values after camera calibration
    if (abs(tag_x) < 3.0)
    {
        set_docking_state("docked");
    }

    else if (abs(tag_x) < 6.0)
    {
        set_docking_state("final_approach");
    }

    else if (abs(tag_x) > 9.0)
    {
        set_docking_state("approach");
    }

    ready_last_pose = false;
}

void DockingController::docked_state_func()
{
    turtlebot_stop();

    gazebo_charge_battery_client();
}

/*** Turtlebot3 Movement Functions ***/
void DockingController::turtlebot_stop()
{
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;

    vel_publisher->publish(cmd_vel_msg);
}

void DockingController::turtlebot_move(double velocity, double radians)
{
    cmd_vel_msg.linear.x = velocity;
    cmd_vel_msg.angular.z = radians;

    vel_publisher->publish(cmd_vel_msg);
}

void DockingController::turtlebot_turn(double radians)
{
    cmd_vel_msg.angular.z = radians;

    vel_publisher->publish(cmd_vel_msg);
}

void DockingController::turtlebot_forward(double velocity)
{
    cmd_vel_msg.linear.x = velocity;
    cmd_vel_msg.angular.z = 0;

    vel_publisher->publish(cmd_vel_msg);
}

/*** Set & Get Functions ***/
void DockingController::set_docking_state(std::string new_docking_state)
{
    
    if (docking_state != new_docking_state)
    {
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(),"Docking state: %s, Last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void DockingController::set_action_state(std::string new_action_state)
{
    if (action_state != new_action_state)
    {
        last_action_state = action_state;
        action_state = new_action_state;
        RCLCPP_INFO(get_logger(),"Action state: %s, Last state: %s", action_state.c_str(), last_action_state.c_str());
    }
}

void DockingController::get_last_pose()
{
    if (ready_pose)
    {
        tf2::Quaternion q(
            last_pose.orientation.x,
            last_pose.orientation.y,
            last_pose.orientation.z,
            last_pose.orientation.w
        );
        q.normalize();
            
        tf2::Matrix3x3 m(q);
        
        double roll, pitch, yaw;
        m.getRPY(roll,pitch,yaw);
        
        tag_x = last_pose.position.x;
        tag_y = last_pose.position.y;
        tag_yaw = yaw;

        // RCLCPP_INFO(get_logger(),"x position: %f", abs(tag_x));
        // RCLCPP_INFO(get_logger(),"y position: %f", abs(tag_y));
        // RCLCPP_INFO(get_logger(),"yaw position: %f", abs(tag_yaw));

        ready_pose = false;     // Wait for next AprilTag Pose
        ready_last_pose = true; // Last AprilTag pose ready
    }

}

void DockingController::publish_state()
{
    docking_interfaces::msg::CurrentState current_state;
    
    current_state.docking_state = docking_state;
    current_state.action_state = action_state;
    state_publisher->publish(current_state);
}

void DockingController::gazebo_charge_battery_client()
{
    auto client = this->create_client<docking_interfaces::srv::GazeboChargeBattery>("gazebo_battery_state/charge_battery_service");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for Battery Charging server to be up...");
    }

    auto request = std::make_shared<docking_interfaces::srv::GazeboChargeBattery::Request>();
    request->command = "charge";

    auto future = client->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Charging Successful.");
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }

    
}



/*** Main Run Function ***/
void DockingController::run()
{
    get_last_pose();            // Get last pose of AprilTag
    docking_state_manager();    // Manage docking states
    action_state_manager();     // Manage action states
    publish_state();            // Publish state to /current_state topic
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockingController>(); 
    
    rclcpp::Rate rate(30.0);

    node->set_docking_state("");

    while (rclcpp::ok())
    {
        node->run();
        rclcpp::spin_some(node);
        //rclcpp::spin(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
