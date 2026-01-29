#ifndef ARMOR_SERIAL_DRIVER_NODE_HPP_
#define ARMOR_SERIAL_DRIVER_NODE_HPP_

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h" 
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "auto_aim_interfaces/msg/gimbal_ctrl.hpp" 
#include "auto_aim_interfaces/msg/gimbal_fdb.hpp" 
#include "auto_aim_interfaces/msg/fdb.hpp"

namespace rm_auto_aim
{

class SerialDriverNode : public rclcpp::Node
{
private:
  double current_v;
  double s_bias;
  double z_bias;
  double pitch_bias;
  double k;
  int16_t vx_=0;
  int16_t vy_=0;
  int16_t vw_=0;

  // 发送数据结构体
  vision_rx_t vision_rx_data_;
  fdb_rx_t fdb_rx_data_;

  float timestamp_offset_;
  std::shared_ptr<::robomaster::SerialDevice> device_ptr_;
  std::unique_ptr<uint8_t[]> recv_buff_;
  std::unique_ptr<uint8_t[]> send_buff_;
  const unsigned int BUFF_LENGTH = 21;//buff length
  const unsigned int BUFF_LENGTH_SEND = 23;//buff length

  // 接受/发送的数据帧头
  frame_header_struct_t frame_recv_header_;
  frame_header_struct_t frame_send_header_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<auto_aim_interfaces::msg::GimbalCtrl>::SharedPtr gimbal_ctrl_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Fdb>::SharedPtr fdb_sub_;

  // 创建tf广播器，用于广播world到gimbal的变换
  std::shared_ptr<tf2_ros::TransformBroadcaster> gimbal_tf_broadcaster;

  // 创建发布器，用于发布弹道轨迹MarkerArray消息对象
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_pub_;

  // 创建发布器，用于发布云台信息
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFdb>::SharedPtr gimbal_fdb_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalFdb>::SharedPtr gimbal_fdb_pub_base_;
  /**
   * 发送数据包
  */

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  uint16_t senderPackSolveEOF(uint8_t *data, uint16_t data_length, uint8_t *send_buf);

  /**
   * 帧头校验
  */
  void searchFrameSOF(uint8_t *frame, uint16_t total_len);

  /**
   * 发布弹道轨迹数据
  */
  void publishTrajectoryMarker(float pitch, float yaw);


  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
public:
  SerialDriverNode(const rclcpp::NodeOptions & options);

  /**
   * 读取串口数据回调函数
  */
  void timerCallback();

  /**
   * 发送云台控制数据回调函数
  */
  void gimbalCtrlCallback(const auto_aim_interfaces::msg::GimbalCtrl::SharedPtr msg);
  void naviCallback(const auto_aim_interfaces::msg::Fdb::SharedPtr msg);
};

}  // namespace rm_auto_aim

#endif  // ARMOR_SERIAL_DRIVER_NODE_HPP_