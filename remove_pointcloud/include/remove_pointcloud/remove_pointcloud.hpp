
#include <iostream>
#include <chrono>
#include <queue>
#include <memory>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/empty.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h> //toRosMsg fromRosMsg
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


namespace taurus
{
const int ACC_POINT_NUM = 3;
class RemovePointcloud : public rclcpp::Node{
public:
    RemovePointcloud(const rclcpp::NodeOptions &options);
    ~RemovePointcloud();
private:
    //pub
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_frame_pointcloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_point_cloud_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cough_point_cloud_pub_;

    //sub
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr livox_lidar_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr range_sensor_sub_;

    //fun
        bool RosInit();
        bool ParamInit();
        pcl::PointCloud<pcl::PointXYZ>::Ptr LoadPointcloudFromPcd(const std::string &filemname);
        void LivoxLidarSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void ComparePointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
        void tfCheckThread();
        //msg
        sensor_msgs::msg::PointCloud2 pcd_map_;
        geometry_msgs::msg::TransformStamped static_map_to_odom_;
        geometry_msgs::msg::TransformStamped tf_livox_to_map;
        //ordinary
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
        // std::string filename_ = "/home/auto/Navi_PCD/test_home_2.pcd"; //tucao
        // std::string filename_ = "/home/auto/Navi_PCD/test_home_change.pcd"; //tucaoduimian
        std::string filename_ = "/home/auto/Navi_PCD/RM2025_55w.pcd"; //RMUC
        // std::string filename_ = "/home/auto/Navi_PCD/new_world.pcd"; //new world

        rclcpp::Time livox_lidar_stamp_;
        bool is_first_frame_ = true;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr previous_livox_lidar_pointcloud_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer_static_;
        double dis_diff_ = 0.0;
        std::thread tf_check_thread_;
        std::queue<pcl::PointCloud<pcl::PointXYZ>> pointcloud_queue;
        std::deque<pcl::PointCloud<pcl::PointXYZ>> pointcloud_deque;
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud;

    //client
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
    
};   
} // namespace tartus

