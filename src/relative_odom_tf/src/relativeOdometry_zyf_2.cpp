#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

std::string base_link = "base_link";
nav_msgs::Odometry baseOdometry; // 存储第一帧消息作为基准
bool firstMessageReceived = false;

ros::Publisher *pubOdometryPointer = NULL;
tf::StampedTransform transformToMap;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

void OdometryHandler(const nav_msgs::Odometry::ConstPtr &odometry)
{
  nav_msgs::Odometry tmp_odom;

  if (!firstMessageReceived) {
    // 存储第一帧消息作为基准
    baseOdometry = *odometry;
    firstMessageReceived = true;
    ROS_INFO("First odometry message received and set as base.");
    return;
  }

  // 计算相对偏移
  double dx = odometry->pose.pose.position.x - baseOdometry.pose.pose.position.x;
  double dy = odometry->pose.pose.position.y - baseOdometry.pose.pose.position.y;
  double dz = odometry->pose.pose.position.z - baseOdometry.pose.pose.position.z;

  tmp_odom.pose.pose.position.x = dx;
  tmp_odom.pose.pose.position.y = dy;
  tmp_odom.pose.pose.position.z = dz;
  tmp_odom.pose.pose.orientation = odometry->pose.pose.orientation;

  // 创建一个transformToMap变换，将里程计信息应用于地图坐标系的点云数据。这个变换包括了位置和方向的信息。
  transformToMap.setOrigin(tf::Vector3(dx, dy, dz));
  transformToMap.setRotation(tf::Quaternion(
      odometry->pose.pose.orientation.x, 
      odometry->pose.pose.orientation.y, 
      odometry->pose.pose.orientation.z, 
      odometry->pose.pose.orientation.w
  ));

  // 发布map到sensor_at_scan的tf关系
  transformToMap.stamp_ = odometry->header.stamp;
  transformToMap.frame_id_ = "odom";
  transformToMap.child_frame_id_ = "base_link";
  // tfBroadcasterPointer->sendTransform(transformToMap);

  // tmp_odom.header.frame_id = "odom";
  // tmp_odom.header.stamp = odometry->header.stamp;
  // tmp_odom.child_frame_id = "base_link";
  // tmp_odom.twist.twist = odometry->twist.twist;
  // tmp_odom.pose.covariance = odometry->pose.covariance;
  // pubOdometryPointer->publish(tmp_odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("base_link", base_link);

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("novatel/oem7/odom", 5, OdometryHandler);

  // ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>("relative_odom", 5);
  // pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::spin();

  return 0;
}