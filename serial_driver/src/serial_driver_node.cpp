#include "serial_driver/serial_driver_node.hpp"
#include <geometry_msgs/msg/twist.hpp>

namespace rm_auto_aim
{

SerialDriverNode::SerialDriverNode(const rclcpp::NodeOptions& options) : Node("serial_driver", options)//串口通信节点
{
  RCLCPP_INFO(this->get_logger(), "Starting SerialDriverNode!");

  std::string device_name = this->declare_parameter("device_name", "/dev/robomaster");
  int baud_rate = this->declare_parameter("baud_rate", 921600);                       //波特率，衡量数据传输速率的单位，表示每秒传输多少符号，921600为高速串口通信
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);               //时间戳偏移，校准时间戳
  current_v = this->declare_parameter("current_v",25.0);                              //当前速度
  s_bias = this->declare_parameter("s_bias",0.03);                                    //水平位置偏置
  z_bias = this->declare_parameter("z_bias",0.0);                                     //高度偏置
  pitch_bias = this->declare_parameter("pitch_bias",0.005);                           //俯仰角偏置
  k = this->declare_parameter("k",0.038);                                             //空气阻力偏置
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    rclcpp::QoS(10),
    std::bind(&SerialDriverNode::cmdVelCallback,this,std::placeholders::_1)
  );



  //由device_ptr_接管串口相关初始化与协议解包功能
  device_ptr_ = std::make_shared<::robomaster::SerialDevice>(device_name, baud_rate);  //创建智能共享指针device_ptr_，用于创建 SerialDevice 对象的共享所有权指针
  if (!device_ptr_->Init())                                                           //共享指针初始化失败的情况
  {
    // return false;
    RCLCPP_ERROR(this->get_logger(), "fail to init SerialDevice!");
  }
  //接收缓冲区和发送缓冲区，在hpp
  recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
  send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH_SEND]);  // debug

  memset(&frame_recv_header_, 0, sizeof(frame_header_struct_t));                      //将结构体内存清零，避免未初始化的内存
  memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));
  //TF变换广播器创建
  gimbal_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);  // test_common

  gimbal_fdb_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalFdb>("/gimbal_fdb", 10);//创建云台反馈数据发布器，发布tf变换信息，队列为10

  gimbal_ctrl_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalCtrl>(  //云台控制订阅
      "gimbal_ctrl", rclcpp::SensorDataQoS(),                                           //Qos配置，定义消息如何从发布者传递到订阅者的规则集合
      std::bind(&SerialDriverNode::gimbalCtrlCallback, this, std::placeholders::_1));   //绑定成员函数

  fdb_sub_ = this->create_subscription<auto_aim_interfaces::msg::Fdb>(                //反馈订阅
      "fdb", rclcpp::SensorDataQoS(),
      std::bind(&SerialDriverNode::naviCallback, this, std::placeholders::_1));
  // initial timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1)),
                                   std::bind(&SerialDriverNode::timerCallback, this));//1ms定时器

  trajectory_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trojectory_markers", 10);//弹道可视化发布者，队列为10
}

void SerialDriverNode::gimbalCtrlCallback(const auto_aim_interfaces::msg::GimbalCtrl::SharedPtr msg)//云台控制回调函数
{
  // RCLCPP_INFO_STREAM(this->get_logger(), std::oct << "=======================" << std::endl
  //                                                 << "Get GimbalCtrl msg!!!!!" << std::endl
  //                                                 << "----------------------" << std::endl
  //                                                 << "yaw: " << msg->yaw << std::endl
  //                                                 << "pitch: " << msg->pit << std::endl
  //                                                 << "dis: " << msg->dis << std::endl
  //                                                 << "tof: " << msg->tof << std::endl
  //                                                 << "pos: " << msg->pos << std::endl
  //                                                 << "empty: " << msg->empty << std::endl;);
  //基础参数赋值
  vision_rx_data_.data.yaw = msg->yaw;
  vision_rx_data_.data.pit = msg->pit;
  vision_rx_data_.data.shoot_yaw_tole = msg->shoot_yaw_tole;
  vision_rx_data_.data.tof = msg->fire_flag;
  vision_rx_data_.data.pos = msg->pos;
  vision_rx_data_.data.empty = msg->empty;
  //msg->x = -16;
  vision_rx_data_.data.base_dx = (msg->x > 0?msg->x:-msg->x); //x坐标的绝对值

  vision_rx_data_.data.vx_ = vx_;
  vision_rx_data_.data.vy_ = vy_;
  vision_rx_data_.data.vw_ = vw_;


  // tempData = msg->cnt;
  // tempData<<1;
  // tempData = tempData && msg-
  vision_rx_data_.data.flag = ((((msg->cnt & 0x1f)) | ((msg->x > 0?1:0) << 5) | (msg->ist_flag << 6)) | (msg->aim_flag << 7));//标志位组装
  vision_rx_data_.data.eof = VISION_EOF;  // EOF：帧结束标识，告诉接收方数据包到此结束

  uint16_t send_length = senderPackSolveEOF((uint8_t*)&vision_rx_data_, sizeof(vision_rx_t), send_buff_.get());//数据打包，将源数据打包，添加协议头、处理EOF转义，返回打包后的长度

  device_ptr_->Write(send_buff_.get(), send_length);                                    //串口发送
}

void SerialDriverNode::timerCallback()                                                      //时间回调函数
{
  static int a = 0;
  static int flag = 0;
  static int last_len = 0;
  static int temp_frame_length = 25;  // frame 's length

  last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, temp_frame_length);//接收串口通信数据

  if (last_len > 1)                                                                           //判断数据是否过长
  {
    RCLCPP_ERROR(get_logger(), "too long msg!!! length: %d", last_len);  //, length: %d",temp_frame_length);
            std::cout<<"+++++++++++++++++:"  <<std::endl;//debug
  }
  else if (last_len == 0)                                                                     //判断是否为无效数据
  {
    RCLCPP_ERROR(get_logger(), "wrong msg!!!");
  }

  while (flag == 0 && last_len == 1)
  {
    // RCLCPP_INFO(get_logger(),"ciiiiiiiiiiiiiiii");//debug
    // std::cout << "ciiiiiiiiiiiiiiii" << std::endl;//debug
    if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF))                         //判断a处的帧标志是否与a+1处的帧标志相同
    {
      flag = 1;
      searchFrameSOF(recv_buff_.get(), a);                                                      //147行
    }
    a++;
  }
  flag = 0;
  a = 0;

  usleep(1);                                                                                    //休眠
}

void SerialDriverNode::naviCallback(const auto_aim_interfaces::msg::Fdb::SharedPtr msg)         //导航回调函数
{
  RCLCPP_INFO(this->get_logger(),"a:%f",msg->yaw);                                              //日志输出
  fdb_rx_data_.data.yaw = msg->yaw;                                                             //串口通信数据赋值
  fdb_rx_data_.data.pitch = msg->pitch;
  std::cout<<"yaw:"<<msg->yaw<<std::endl<<"pitch:"<<msg->pitch<<std::endl;
  // fdb_rx_data_.data.yaw = 0;
  // fdb_rx_data_.data.pitch = 1;
  // fdb_rx_data_.data.meiyoude = 2;
  fdb_rx_data_.data.eof = VISION_EOF_FDB;  // EOF

  uint16_t send_length = senderPackSolveEOF((uint8_t*)&fdb_rx_data_, sizeof(fdb_rx_t), send_buff_.get());

  device_ptr_->Write(send_buff_.get(), send_length);
}
uint16_t SerialDriverNode::senderPackSolveEOF(uint8_t* data, uint16_t data_length, uint8_t* send_buf)
{
  uint8_t index = 0;

  memcpy(send_buf + index, data, data_length);

  // Append_CRC16_Check_Sum(send_buf, data_length + 9);

  return data_length + 0;
}

void SerialDriverNode::searchFrameSOF(uint8_t* frame, uint16_t total_len)                       //搜寻帧起始，从指定位置开始解析完整的数据帧，传递帧起始的位置
{
  static vision_tx_t vision_tx_data;  // debug receive
  uint16_t i;

  for (i = 0; i < total_len;)
  {
    if (*frame == HEADER_SOF)
    {
      // RCLCPP_INFO(get_logger(), "header get!");  //debug

      memcpy(&vision_tx_data.buff, frame, sizeof(vision_tx_data));                              //复制整一段帧数据

      // pub tf    test_common
      geometry_msgs::msg::TransformStamped transform;                                           //tf变换
      transform.header.stamp = this->get_clock()->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
      transform.header.frame_id = "world";
      transform.child_frame_id = "gimbal_link";
      tf2::Quaternion q;
      vision_tx_data.data.imu_pit = -vision_tx_data.data.imu_pit / 57.29578;
      vision_tx_data.data.imu_yaw = vision_tx_data.data.imu_yaw / 57.29578;
      q.setRPY(0.0, vision_tx_data.data.imu_pit, vision_tx_data.data.imu_yaw);
      transform.transform.rotation = tf2::toMsg(q);

      gimbal_tf_broadcaster->sendTransform(transform);                                                //坐标变换发布

      publishTrajectoryMarker(vision_tx_data.data.imu_pit, vision_tx_data.data.imu_yaw);              //203行

      auto_aim_interfaces::msg::GimbalFdb gimbal_fdb;
      gimbal_fdb.imu_pit = vision_tx_data.data.imu_pit;                                               //云台俯仰角
      gimbal_fdb.imu_pit_spd = vision_tx_data.data.imu_pit_spd;                                       //云台俯仰角速度
      gimbal_fdb.imu_yaw = vision_tx_data.data.imu_yaw;                                               //云台偏航角
      gimbal_fdb.imu_yaw_spd = vision_tx_data.data.imu_yaw_spd;                                       //云台偏航角速度
      gimbal_fdb.camp = vision_tx_data.data.mode_msg.camp;  //机器人阵营  0：红方  1：蓝方
      gimbal_fdb.priority_number = vision_tx_data.data.priority_number;
      gimbal_fdb.aiming_mode = vision_tx_data.data.mode_msg.aiming_mode; //选择相机
      gimbal_fdb.shooter_speed = vision_tx_data.data.shooter_speed; //射速
      gimbal_fdb.lidar_mode = vision_tx_data.data.mode_msg.lidar_mode; //基地血量判断雷达

      //gimbal_fdb.aiming_mode = 2;
      gimbal_fdb_pub_->publish(gimbal_fdb);                                                           //串口通信数据发布
            // std::cout<<gimbal_fdb.aiming_mode<<std::endl;

      i = total_len;
    }
    else                                                                                              //若不是帧头，则向下搜索
    {
      frame++;
      i++;
      // debug
      RCLCPP_WARN(get_logger(), "bad header!");  // debug
    }
  }
}

// publish trajectory marker
void SerialDriverNode::publishTrajectoryMarker(float pitch, float yaw)                                //发布轨迹标志
{
  
  current_v = get_parameter("current_v").as_double();                                                 //从ROS参数服务器获取名为"current_v"的参数值，并转换为double类型，下同
  s_bias = get_parameter("s_bias").as_double();
  z_bias = get_parameter("z_bias").as_double();
  pitch_bias = get_parameter("pitch_bias").as_double();
  k = get_parameter("k").as_double();

  pitch +=pitch_bias;
  // 创建MarkerArray消息对象
  auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();

  // 创建线条标记...
  auto line_strip_marker = std::make_unique<visualization_msgs::msg::Marker>();                       //创建智能指针管理的Marker对象
  line_strip_marker->header.frame_id = "world";                                                       //设置坐标系
  line_strip_marker->type = visualization_msgs::msg::Marker::LINE_STRIP;                              //设置可视化类型为线带
  line_strip_marker->action = visualization_msgs::msg::Marker::ADD;
  line_strip_marker->scale.x = 0.01;                                                                  //线宽
  line_strip_marker->pose.orientation.w = 1.0;                                                        //姿态
  line_strip_marker->color.a = 1.0;                                                                   //不透明
  line_strip_marker->color.r = 1.0;                                                                   //颜色

  // 生成弹道点...
  const double GRAVITY = 9.78;
  
  for (int t = 0; t <= 20; t++)
  {
    const double t0 = static_cast<double>(t) / 10.0;
    const double s = (std::log(t0 * k * current_v * std::cos(-pitch)) / k)+s_bias;
    const double x = s * std::cos(yaw);
    const double y = s * std::sin(yaw);
    const double z = (current_v * std::sin(-pitch) * t0 - GRAVITY * t0 * t0 / 2.0)+z_bias;
    // RCLCPP_INFO_STREAM(this->get_logger(), "position :" << std::endl
    //                         << "x: " << x << std::endl
    //                         << "y: " << y << std::endl
    //                         << "z: " << z << std::endl
    //                         << "t0: " << t0 << std::endl
    //                         <<"speed_: " << speed  << std::endl
    //                         << "speed_z: " << speed * std::sin(-pitch) << std::endl);

    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    line_strip_marker->points.push_back(point);
  }

  // 将标记添加到MarkerArray消息对象中...
  marker_array_msg->markers.push_back(*line_strip_marker);

  trajectory_marker_pub_->publish(std::move(marker_array_msg));
}


void SerialDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx_ = static_cast<int16_t>(msg->linear.x *1000.0);
  vy_ = static_cast<int16_t>(msg->linear.y *1000.0);
  vw_ = static_cast<int16_t>(msg->angular.z *1000.0);

  vx_ = std::clamp(vx_,(int16_t)-3000.0,(int16_t)3000);
  vy_ = std::clamp(vy_,(int16_t)-3000.0,(int16_t)3000);
  vw_ = std::clamp(vw_,(int16_t)-3000.0,(int16_t)3000);

  RCLCPP_INFO(this->get_logger(), "Received velocity: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f", 
              msg->linear.x, msg->linear.y, msg->angular.z);

}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::SerialDriverNode)
