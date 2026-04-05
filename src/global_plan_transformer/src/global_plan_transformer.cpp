#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class GlobalPlanTransformer {
public:
    GlobalPlanTransformer() : tf_listener_(tf_buffer_) {
        // 订阅 DWAPlannerROS 生成的路径
        path_sub_ = nh_.subscribe("move_base/DWAPlannerROS/global_plan", 1, &GlobalPlanTransformer::pathCallback, this);
        // 发布转换后的路径
        path_pub_ = nh_.advertise<nav_msgs::Path>("transformed_global_plan", 1);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("The transformer received empty path");
            return;
        }

        // 获取 odom -> map 的变换
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform("map", "odom", ros::Time(0), ros::Duration(0.5));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not find tf: map->odom  %s", ex.what());
            return;
        }

        // 创建转换后的路径
        nav_msgs::Path transformed_path;
        transformed_path.header.stamp = ros::Time::now();
        transformed_path.header.frame_id = "map";  // 目标坐标系

        // 逐个转换路径点
        for (const auto& pose : msg->poses) {
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(pose, transformed_pose, transformStamped);
            transformed_path.poses.push_back(transformed_pose);
        }

        // 发布转换后的路径
        path_pub_.publish(transformed_path);
        ROS_INFO("Published transformed global plan, it has %lu poses.", transformed_path.poses.size());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_plan_transformer");
    GlobalPlanTransformer transformer;
    ros::spin();
    return 0;
}
