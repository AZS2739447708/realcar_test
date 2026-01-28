#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#define HEADER_SOF 0x11//0xA5
#define END1_SOF 0x22//0x0D
#define END2_SOF 0x33//0x0A
#define VISION_EOF 0xcc
#define VISION_EOF_FDB 0xdd

#pragma pack(push, 1)

typedef struct
{
  int16_t vx;
  int16_t vy;
  int16_t vw;
} vision_rx_data_t;


typedef enum
{
  GAME_STATUS_FDB_ID  = 0x0001,
  CHASSIS_ODOM_FDB_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
  GIMBAL_CMD_ID       = 0x0103,
  TEST_GIMBAL_CMD_ID  = 0x0000,//just test, this is useless
  MAP_FDB_ID          = 0x0104
} referee_data_cmd_id_type;

typedef struct {
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

// chassis control message
typedef struct {
  float vx;
  float vy;
  float vw;
  uint8_t super_cup;
	
}  chassis_ctrl_info_t;

// chassis odom feedback message
typedef struct {
  
  float x_fdb;
  float y_fdb;
  float vx_fdb;
  float vy_fdb;
  float vw_fdb;
  float gimbal_yaw_fdb;
  float gimbal_pitch_fdb;
  float gimbal_yaw_imu;
  float gimbal_yaw_rate_imu;
  float gimbal_pitch_imu;
  float gimbal_pitch_rate_imu;
	float super_cup_state;

}  chassis_odom_info_t;

// gimbal control message
typedef struct {

	float gimbal_yaw_cmd;
	float gimbal_pitch_cmd;
	float shoot_cmd;
	float friction_cmd;
  
  float aiming_flag;
  float pit;
  float yaw;
  float dis;
  float tof;

  float connect_status;       //连接状态： 0（状态异常） 1（正常连接） 2（正常且执行指令中）
  float process_status;       //流程状态： 0（空闲）  1（进行中） 2（导航中） 3（完成）
  float cmd_mode;             //执行指令： 0（空闲）  1 (退回哨兵巡逻区）2（前往防守前哨站）3（前往我方占领区）4(前往敌方占领区）5(进攻敌方前哨站) 6(进攻敌方基地） 
  float spinning_mode;        //小陀螺状态：  0（小陀螺关闭）  1（小陀螺关闭）                     
  float navigation_staus;     //导航状态：    0(空闲) 1（已到达）2（运行中） 3（规划失败）
  float location_staus;       //定位状态:     0(正常) 1（漂移） 2（重定位中）
  float defend_mode;          //防御模式：    0（无敌状态） 1（无敌状态解除）  //我方前哨站击毁前 自动哨兵处于无敌状态
  float auto_aim;             //自动瞄准状态： 0（空闲） 1（索敌中）2（识别到敌方机器人）3（射击中）4（自瞄异常）
  float robot_x;              //自动哨兵位置X
  float robot_y;              //自动哨兵位置Y
  float robot_vx;             //自动哨兵速度vx
  float robot_vy;             //自动哨兵速度vy
  float robot_vw;             //自动哨兵速度vw
	float robot_gyro_yaw;       //哨兵云台角度yaw
  
}  gimbal_ctrl_info_t;


typedef struct {
	float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
	float gyro_z;
	float gyro_yaw;
	
} message_info_t;

typedef struct {
  
  float game_type;
  float game_progress;
  float shooter_cooling_rate;
  float shooter_speed_limit;
  float shooter_cooling_limit;
  float robot_id;
  float remain_HP;
  float max_HP;
  float armor_id;
  float hurt_type;
  float bullet_freq;
  float bullet_speed;
  float bullet_remaining_num;
  float key_board;
  float other_robot_id;
  float tgx;
  float tgy;
  float tgz;
  float game_time;
  float enemy_sentry_HP;
  float enemy_outpost_HP;
  float self_outpost_HP;
} robot_status_t;

typedef union{
  uint8_t buff[22];

  struct{
    uint8_t SOF;
    float imu_pit;
    float imu_yaw;
    float imu_pit_spd;
    float imu_yaw_spd;

    struct
    {
      uint8_t vacancy :1;
      uint8_t camp    :1;
      uint8_t aiming_mode: 3;
      uint8_t lidar_mode: 3;
    }mode_msg;
    uint8_t shooter_speed;
    uint8_t priority_number;
    uint8_t EOF1;
    uint8_t EOF2;
  }data;
}vision_tx_t;

typedef union{
  uint8_t buff[25];//源神从23改成25
  struct{
    float yaw;
    float pit;
    float shoot_yaw_tole;
    float tof;
    float pos;
    uint16_t base_dx;//吊射用
    uint8_t empty;
    uint8_t flag;

    int16_t vx_;
    int16_t vy_;
    int16_t vw_;

    uint8_t eof;
  }data;
}vision_rx_t;

typedef union{
  uint8_t buff[17];//源神从23改成25
  struct{
    double yaw;
    double pitch;
    double meiyoude;
    uint8_t eof;
  }data;
}fdb_rx_t;
#pragma pack(2)
typedef struct 
{
  uint8_t intention;
  uint8_t start_position_x;
  uint8_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
} map_data_t;
#pragma pack()





#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
