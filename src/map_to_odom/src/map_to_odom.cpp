#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

double originOffsetX = 0.0;
double originOffsetY = 0.0;
double originOffsetZ = 0.0;

nav_msgs::Odometry odometryIn;
std::string base_link = "base_link";
std::string odom_frame = "odom";
std::string map_frame = "map";

tf::TransformBroadcaster *tfBroadcasterPointer = nullptr;
tf::TransformListener *tfListenerPointer = nullptr;

// 处理里程计数据，动态生成 map -> base_link 的变换
void OdometryHandler(const nav_msgs::Odometry::ConstPtr &odometry)
{
    // // 读入odom数据
    // odometryIn = *odometry;
    
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

    // 获取 odom -> base_link 的变换
    tf::StampedTransform transform_odom_to_base;
    try
    {
        tfListenerPointer->lookupTransform(odom_frame, base_link, ros::Time(0), transform_odom_to_base);

        // 计算 map -> odom 的变换：map -> odom = (map -> base_link) * (base_link -> odom)^-1
        tf::Transform transform_map_to_odom = transform_map_to_base * transform_odom_to_base.inverse();

        // 发布 map -> odom 的 tf 变换
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = odometry->header.stamp;
        msg.header.frame_id = map_frame;
        msg.child_frame_id = odom_frame;

        // 填充转换数据
        msg.transform.translation.x = transform_map_to_odom.getOrigin().x();
        msg.transform.translation.y = transform_map_to_odom.getOrigin().y();
        msg.transform.translation.z = transform_map_to_odom.getOrigin().z();

        tf::Quaternion q = transform_map_to_odom.getRotation();
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();

        tfBroadcasterPointer->sendTransform(msg);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_to_odom");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // 获取参数
    nhPrivate.getParam("base_link", base_link);
    nhPrivate.getParam("originOffsetX", originOffsetX);
    nhPrivate.getParam("originOffsetY", originOffsetY);
    nhPrivate.getParam("originOffsetZ", originOffsetZ);
    nhPrivate.param<std::string>("odom_frame", odom_frame, "odom");
    nhPrivate.param<std::string>("map_frame", map_frame, "map");

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
