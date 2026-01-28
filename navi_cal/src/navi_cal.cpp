#include "navi_cal/navi_cal.hpp"
namespace taurus
{
    Navi_cal::Navi_cal(const rclcpp::NodeOptions &options) : Node("Navi_cal",options)   //const(常量）：只读引用，不能修改
    {   
        // RCLCPP_INFO(this->get_logger(),"Navi_cal");
        auto period = std::chrono::milliseconds(40);                                        //以25Hz执行timerCallback函数
        timer_ = this->create_wall_timer(period,std::bind(&Navi_cal::timerCallback,this));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());                  //tf变换
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);     //tf变换
        fdb_pub_ = this->create_publisher<auto_aim_interfaces::msg::Fdb>("/fdb", 10);      //自定义发布者auto_aim_interfaces
        gimbal_fdb_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalFdb>(     
        "gimbal_fdb", rclcpp::SensorDataQoS(),
        std::bind(&Navi_cal::GimbalFdbCallback, this, std::placeholders::_1));     //接收者，接受云台反馈数据GimbalFdb，用于获取平台当前的信息
        ParamInit();                                                                //80行
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Navi_cal::paramCallback, this, std::placeholders::_1));   //paramCallback函数85行，
    }
    Navi_cal::~Navi_cal()
    {
    }
    void Navi_cal::GimbalFdbCallback(const auto_aim_interfaces::msg::GimbalFdb::SharedPtr msg)   //云台反馈回调函数
    {
        lidar_mode = msg->lidar_mode;                                            //雷达模式
        aiming_mode = msg->aiming_mode;                                          //瞄准模式
    }
    float Navi_cal::pitchTrajectoryCompensation(float s, float z, float v)       //俯仰轨迹补偿函数，应该是针对弹道的补偿
    {   
    auto dist_vertical = z;                                                     //目标高度
    auto vertical_tmp = dist_vertical;                                          //更新临时高度数据
    auto dist_horizonal = s;                                                    //水平距离
    auto pitch = atan(dist_vertical / dist_horizonal) * 180 / PI;               //初始俯仰角度
    auto pitch_new = pitch;                                                     //更新新的俯仰角度
    for (int i = 0; i < 10; i++)
    {
        auto x = 0.0;
        auto y = 0.0;
        auto p = tan(pitch_new / 180 * PI);                                     //p是俯仰角度
        auto u = v / sqrt(1 + pow(p, 2));                                       //u为速度水平分量
        auto delta_x = dist_horizonal / R_K_iter;                               //R_K_iter在hpp里，为10，将水平距离分为R_K_iter份
        for (int j = 0; j < R_K_iter; j++)                                      //弹道微分方程，四阶龙格-库塔法（RK4）求解考虑空气阻力的弹道方程
        {
            auto k1_u = -k * u * sqrt(1 + pow(p, 2));                           //k在hpp里，double k = 0.000556;
            auto k1_p = -GRAVITY / pow(u, 2);                                   //GRAVITY在hpp，重力加速度，为9.78
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -GRAVITY / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -GRAVITY / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * delta_x;  
            auto k3_p_sum = p + k3_p * delta_x;  

            auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -GRAVITY / pow(k3_u_sum, 2);

            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);           //RK4加权平均
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x += delta_x;                                                       //水平位置累积，计算子弹的水平飞行距离
            y += p * delta_x;                                                   //垂直位置累积
        }
        auto error = dist_vertical - y;                                             //计算精度
        if (abs(error) <= 0.0005)                                                   //若精度小于0.0005则退出循环
        {
            break;
        }
        else
        {
            vertical_tmp += error;                                                  //否则高度补偿
            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / PI;             //俯仰角补偿
        }
    }
    return pitch_new;
    }
    void Navi_cal::ParamInit()                                          //ParamInit函数
    {
        this->declare_parameter<double>("pit_bias", 0.0);               //声明参数的格式，double类型的pit_bias，默认值为0.0，declare_parameter的作用是在节点中声明一个函数
        this->get_parameter_or<double>("pit_bias", pit_bias, 0.0);      //get_parameter_or的作用是若平台有输出，则将输出值赋值给pit_bias,若没有则赋值0.0
    }
    rcl_interfaces::msg::SetParametersResult Navi_cal::paramCallback(   //paramCallback函数，回调函数
        const std::vector<rclcpp::Parameter> &params)                   //const std::vector<rclcpp::Parameter> 类型参数 &params；&是引用传递，避免复制；当参数发生改变时，params可以获取改变参数的向量
    {
        rcl_interfaces::msg::SetParametersResult result;                //定义result
        result.successful = true;                                       //假设result中的所有参数设置都成功，遇到问题再设置为失败
        
        for (const auto &param : params) {                              //一个参数改变，可能导致多个参数同时改变，循环处理
            if (param.get_name() == "pit_bias") {                       //获取param中改变参数的名字是否为我们要处理的参数pit_bias
                pit_bias = param.as_double();                           //param.as_double()：将参数值转换为double类型
                RCLCPP_INFO(this->get_logger(), "参数更新: pit_bias = %.3f", pit_bias);  //输出信息级别日志，方便调试和监控参数变化
            }
        }
        return result;                                                  //返回结果true/false
    }

    void Navi_cal::timerCallback()                                      //时间回调函数
    {
        RCLCPP_INFO(this->get_logger(),"timerCallback");
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","lidar_link",tf2::TimePointZero);//将lidar_link坐标系变到map坐标系
            lidar_pose_.position.x = transform.transform.translation.x;     //获取坐标
            lidar_pose_.position.y = transform.transform.translation.y;
            lidar_pose_.position.z = transform.transform.translation.z;
            tf2::Quaternion q(                                          //声明四元数q
                transform.transform.rotation.x,                         //从TF变换中创建四元数对象
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            //将四元数转换为欧拉角
            tf2::Matrix3x3 m(q);                                         //创建旋转矩阵
            m.getRPY(lidar_roll_,lidar_pitch_,lidar_yaw_);              //从旋转矩阵提取滚转(roll)、俯仰(pitch)、偏航(yaw)角
            RCLCPP_INFO(this->get_logger(),"lidar yaw: %f, lidar x: %f, lidar y: %f,lidar z: %f,",lidar_yaw_,lidar_pose_.position.x,lidar_pose_.position.y,lidar_pose_.position.z);//输出日志
        }catch(tf2::TransformException  &ex){                           //异常处理，ex 是捕获的异常对象引用，ex.what() 返回错误描述字符串
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());    
        }
        try{                                                            //这里是机器人位姿变换到map，原理与上文一样
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","cloud_link",tf2::TimePointZero);
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            robot_pose_.position.z = transform.transform.translation.z;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(robot_roll_,robot_pitch_,robot_yaw_);
            RCLCPP_INFO(this->get_logger(),"Current yaw: %f, Current x: %f, Current y: %f,Current z: %f,",robot_yaw_,robot_pose_.position.x,robot_pose_.position.y,robot_pose_.position.z);
        }catch(tf2::TransformException  &ex){ 
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }
        // fdb_msg_.yaw = 1;
        fdb_msg_.pitch = 2;
        map_yaw_ = atan2((target_y_ - robot_pose_.position.y),(target_x_ - robot_pose_.position.x)) * 180 /3.1415926;
        //基地
        target_dis_ = sqrt((target_y_ - robot_pose_.position.y) * (target_y_ - robot_pose_.position.y)+(target_x_ - robot_pose_.position.x) * (target_x_ - robot_pose_.position.x));
        if(lidar_mode == 1 || aiming_mode ==2)//low
        { 
            target_z_ = 0.3455;                                                                         //目标高度
            shoot_speed = 15.5;                                                                         //射速
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);  //目的俯仰角

        }
        else//tower
        {
            target_z_ = 0.8855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        if(robot_pose_.position.x >=-0.660237 && robot_pose_.position.x <= 4.069704 && robot_pose_.position.y >= 5.882196 && robot_pose_.position.y <= 8.184546)
        {
            target_z_ = 0.6855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        else
        {
            target_z_ = 0.8855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        robot_yaw_ = robot_yaw_ * 180 /3.1415926;
        result_yaw_ = map_yaw_ - robot_yaw_; //+往左偏，-往右偏'
        fdb_msg_.yaw = result_yaw_;
        fdb_msg_.pitch = result_pit_;
        //INFO
        RCLCPP_INFO(this->get_logger(),"map_yaw_ :%f,",map_yaw_);                       //调试信息输出和日志发布
        RCLCPP_INFO(this->get_logger(),"robot_yaw_ :%f,",robot_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_yaw_ :%f,",result_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_pit_ :%f,",result_pit_);
        RCLCPP_INFO(this->get_logger(),"result_pit_pid:%f,",result_pit_ * PI / 180);
        RCLCPP_INFO(this->get_logger(),"target_dis_ :%f,",target_dis_);
        RCLCPP_INFO(this->get_logger(),"pit_bias :%f,",pit_bias);  
        RCLCPP_INFO(this->get_logger(),"lidar_mode  :%i,",lidar_mode);  
        RCLCPP_INFO(this->get_logger(),"shoot_speed  :%f,",shoot_speed);  
        RCLCPP_INFO(this->get_logger(),"target_z_ :%f,",target_z_);  

        fdb_pub_->publish(fdb_msg_);
    }
}// namespace taurus2weee3
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(taurus::Navi_cal);
//pitchTrajectoryCompensation