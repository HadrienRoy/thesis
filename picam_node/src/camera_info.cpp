#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/camera_info.hpp"


using std::placeholders::_1;



class CameraInfoNode : public rclcpp::Node 
{
    public:
        CameraInfoNode() : Node("camera_info_node") 
        {
            /*** Parameters ***/
            // this->declare_parameter("image_height");
            // this->get_parameter("image_height", image_height);
    
            // this->declare_parameter("image_width");
            // this->get_parameter("image_width", image_width);

            // this->declare_parameter("distortion_model");
            // this->get_parameter("distortion_model", distortion_model);

            // this->declare_parameter("D");
            // this->get_parameter("D", D);

            // this->declare_parameter("K");
            // this->get_parameter("K", K);

            // this->declare_parameter("R");
            // this->get_parameter("R", R);

            // this->declare_parameter("P");
            // this->get_parameter("P", P);


            /*** Publishers Def***/
            camera_info_publisher = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info2", 10);


            timer = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CameraInfoNode::publishCameraInfo, this)
            );

            
            RCLCPP_INFO(this->get_logger(), "Camera Info Node has been started.");
        }

        void publishCameraInfo()
        {
            msg_ci.height = 1280;
            msg_ci.width = 960;
            msg_ci.distortion_model = "plumb_bob";
 
            msg_ci.d[0] = 0.1644274736100891;
            msg_ci.d[1] = -0.2717244716038656;
            msg_ci.d[2] = -0.002867946281892625;
            msg_ci.d[3] = -9.69782173585606e-05;
            msg_ci.d[4] = 0.0;

            msg_ci.k[0] = 999.461170663331;
            msg_ci.k[1] = 0.0;
            msg_ci.k[2] = 642.2582577578172;
            msg_ci.k[3] = 0.0;
            msg_ci.k[4] = 996.9611451866272;
            msg_ci.k[5] = 474.1471906434548;
            msg_ci.k[6] = 0.0;
            msg_ci.k[7] = 0.0;
            msg_ci.k[8] = 1.0;

            msg_ci.r[0] = 1.0;
            msg_ci.r[1] = 0.0;
            msg_ci.r[2] = 0.0;
            msg_ci.r[3] = 0.0;
            msg_ci.r[4] = 1.0;
            msg_ci.r[5] = 0.0;
            msg_ci.r[6] = 0.0;
            msg_ci.r[7] = 0.0;
            msg_ci.r[8] = 1.0;

            msg_ci.p[0] = 1022.143981933594;
            msg_ci.p[1] = 0.0;
            msg_ci.p[2] = 642.2379933862903;
            msg_ci.p[3] = 0.0;
            msg_ci.p[4] = 0.0;
            msg_ci.p[5] = 1021.583557128906;
            msg_ci.p[6] = 471.9362383152657;
            msg_ci.p[7] = 0.0;
            msg_ci.p[8] = 0.0;
            msg_ci.p[9] = 0.0;
            msg_ci.p[10] = 1.0;
            msg_ci.p[11] = 0.0;

            msg_ci.binning_x = 0;
            msg_ci.binning_y = 0;

            msg_ci.roi.x_offset = 0.0;
            msg_ci.roi.y_offset = 0.0;
            msg_ci.roi.height = 0.0;
            msg_ci.roi.width = 0.0;
            msg_ci.roi.do_rectify = false;

            camera_info_publisher->publish(msg_ci);
        }

    private:

        /*** Variables ***/
        rclcpp::TimerBase::SharedPtr timer;
        // int image_height;
        // int image_width;
        // std::string distortion_model;
        // float D[5];
        // float K[9];
        // float R[9];
        // float P[12];
        // int binning_x;
        // int binning_y;

        /*** Messages ***/
        sensor_msgs::msg::CameraInfo msg_ci;

        /*** Publishers Dec ***/
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraInfoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
