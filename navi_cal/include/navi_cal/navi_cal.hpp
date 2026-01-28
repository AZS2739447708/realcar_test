// #include <rclcpp/rclcpp.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include "tf2_ros/transform_broadcaster.h" 
// #include "tf2_ros/transform_listener.h"
// #include <tf2_ros/buffer.h>
// #include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h" 
// #include "tf/transform_datatypes.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/twist.hpp"
#include <mutex>
#include "auto_aim_interfaces/msg/fdb.hpp"
#include "auto_aim_interfaces/msg/gimbal_fdb.hpp" 


#define GRAVITY 9.78
#define PI 3.1415926
typedef unsigned char uint8_t;


namespace taurus
{
class Navi_cal : public rclcpp::Node{
  public:
    Navi_cal(const rclcpp::NodeOptions &options);
    ~Navi_cal();
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Subscription<auto_aim_interfaces::msg::GimbalFdb>::SharedPtr gimbal_fdb_sub_;
    rcl_interfaces::msg::SetParametersResult paramCallback(
        const std::vector<rclcpp::Parameter> &params);
    rclcpp::TimerBase::SharedPtr navier_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Pose lidar_pose_;
    // 基地
    //RMUC
   double target_x_ = 21.617825;// 10 为雷达到地图中心 11为 基地到地图中心
   double target_y_ = 2.406705; // 0.84为地图中心到六边形右上角 0.307 为右上角到雷达
    //xinchangdi
    //  double target_x_ = 20.480222; //18.88  在对面
    //  double target_y_ = 6.161969;


    double target_z_;//基地高度
    double target_dis_;
    double result_yaw_;
    double result_pit_;

    double map_yaw_;
    double shoot_speed;
    double robot_yaw_;
    double robot_roll_;
    double robot_pitch_;
    double lidar_yaw_;
    double lidar_roll_;
    double lidar_pitch_;
    double pit_bias;
    int lidar_mode;
    int aiming_mode;
    auto_aim_interfaces::msg::Fdb fdb_msg_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Fdb>::SharedPtr fdb_pub_;
    // rclcpp::Subscription<auto_aim_interfaces::msg::GimbalFdb>::SharedPtr fdb_sub_;
    int R_K_iter = 10;
    double k = 0.000556;
    //fun
    void ParamInit();
    void timerCallback();
    float pitchTrajectoryCompensation(float s, float z, float v);
    void GimbalFdbCallback(const auto_aim_interfaces::msg::GimbalFdb::SharedPtr msg);

    

};   
} // namespace tartus

