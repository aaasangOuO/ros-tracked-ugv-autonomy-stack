#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
using namespace std;

double originOffsetX = 0.0;
double originOffsetY = 0.0;
double originOffsetZ = 0.0;

nav_msgs::Odometry odometryIn;
std::string base_link = "base_link";
std::string odom_frame = "odom";
std::string map_frame = "map"; 
ros::Publisher current_pose_pub;

tf::TransformBroadcaster *tfBroadcasterPointer = nullptr;
tf::TransformListener *tfListenerPointer = nullptr;

// 处理里程计数据，动态生成 map -> base_link 的变换
void OdometryHandler(const nav_msgs::Odometry::ConstPtr &odometry)
{
    
    // 提取并修正 odom 数据，生成 map -> base_link 的变换
    tf::StampedTransform transform_map_to_base;
    transform_map_to_base.setOrigin(tf::Vector3(
        odometry->pose.pose.position.x - originOffsetX,
        odometry->pose.pose.position.y - originOffsetY,
        odometry->pose.pose.position.z - originOffsetZ));

    transform_map_to_base.setRotation(tf::Quaternion(
        odometry->pose.pose.orientation.x,
        odometry->pose.pose.orientation.y,
        odometry->pose.pose.orientation.z,
        odometry->pose.pose.orientation.w));

    transform_map_to_base.stamp_ = odometry->header.stamp;
    transform_map_to_base.frame_id_ = map_frame;
    transform_map_to_base.child_frame_id_ = base_link;

    try
    {

        // 发布 /current_pose 消息
        geometry_msgs::PoseStamped current_pose_msg;
        current_pose_msg.header.stamp = odometry->header.stamp;
        current_pose_msg.header.frame_id = map_frame;

        current_pose_msg.pose.position.x = transform_map_to_base.getOrigin().x();
        current_pose_msg.pose.position.y = transform_map_to_base.getOrigin().y();
        current_pose_msg.pose.position.z = transform_map_to_base.getOrigin().z();

        tf::Quaternion base_quaternion = transform_map_to_base.getRotation();
        current_pose_msg.pose.orientation.x = base_quaternion.x();
        current_pose_msg.pose.orientation.y = base_quaternion.y();
        current_pose_msg.pose.orientation.z = base_quaternion.z();
        current_pose_msg.pose.orientation.w = base_quaternion.w();

        
        cout << current_pose_msg.pose.orientation.x << endl;

        current_pose_pub.publish(current_pose_msg);

    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_odom_2");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // 获取参数
    nhPrivate.getParam("base_link", base_link);
    nhPrivate.getParam("originOffsetX", originOffsetX);
    nhPrivate.getParam("originOffsetY", originOffsetY);
    nhPrivate.getParam("originOffsetZ", originOffsetZ);
    nhPrivate.param<std::string>("odom_frame", odom_frame, "odom");
    nhPrivate.param<std::string>("map_frame", map_frame, "map");

    // 初始化发布器
    ros::Publisher current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

    // 订阅里程计数据
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("novatel/oem7/odom", 5, OdometryHandler);
    
    

    // 初始化TF广播器和监听器
    tf::TransformBroadcaster tfBroadcaster;
    tfBroadcasterPointer = &tfBroadcaster;

    tf::TransformListener tfListener;
    tfListenerPointer = &tfListener;

    ros::spin();
    return 0;
}



    // #include <ros/ros.h>
    // #include <geometry_msgs/PoseStamped.h>
    // #include <nav_msgs/Odometry.h>

    // ros::Publisher current_pose_pub; // 发布geometry_msgs/PoseStamped的发布者

    // void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    // {
    // // 创建一个PoseStamped消息
    // geometry_msgs::PoseStamped current_pose_msg;

    // // 设置时间戳和框架
    // current_pose_msg.header.stamp = ros::Time::now();
    // current_pose_msg.header.frame_id = "odom"; // 设定框架名称，取决于实际的坐标框架

    // // 将Odometry中的位姿赋值给PoseStamped
    // current_pose_msg.pose = msg->pose.pose;

    // // 发布转换后的PoseStamped消息
    // pose_pub.publish(current_pose_msg);

    // ROS_INFO("Published PoseStamped with position: x = %f, y = %f, z = %f",
    // current_pose_msg.pose.position.x,
    // current_pose_msg.pose.position.y,
    // current_pose_msg.pose.position.z);
    // }

    // int main(int argc, char** argv)
    // {
    // // 初始化ROS节点
    // ros::init(argc, argv, "map_to_odom_2");
    // ros::NodeHandle nh;

    // // 创建订阅者，订阅nav_msgs/Odometry话题
    // ros::Subscriber odom_sub = nh.subscribe("novatel/oem7/odom", 10, odomCallback);

    // // 创建发布者，发布geometry_msgs/PoseStamped话题
    // pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

    // // 保持节点运行
    // ros::spin();

    // return 0;
    // }

