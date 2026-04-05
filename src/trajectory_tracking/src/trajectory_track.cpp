#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <vector>
#include <iostream>

using namespace std;

// double originOffsetX = 0.0;
// double originOffsetY = 0.0;
// double originOffsetZ = 0.0;

// 声明初始位置变量
// double initial_x = 441485.2023248055;
// double initial_y = 4423140.423381293;
// double initial_z = 53.61276412755251;
// double initial_yaw = 0.0;
// bool isFirstFrame_2 = true; // 用于yaw判断是否是第一帧


class TrajectoryTracking {
public: 
    TrajectoryTracking(ros::NodeHandle &nh) :
        nh_(nh), lookahead_distance_(0.2), stop_distance_(0.5), kp_(10), ki_(0.0), kd_(0.01) {

        path_sub_ = nh_.subscribe("transformed_global_plan", 10, &TrajectoryTracking::pathCallback, this);
        odom_sub_ = nh_.subscribe("novatel/oem7/odom", 10, &TrajectoryTracking::odomCallback, this);
        imu_sub_ = nh_.subscribe("gps/imu", 10, &TrajectoryTracking::imuCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel2", 10);

        // nh_.getParam("base_link", base_link_);
        // nh_.getParam("originOffsetX", originOffsetX_);
        // nh_.getParam("originOffsetY", originOffsetY_);
        // nh_.getParam("originOffsetZ", originOffsetZ_);

        last_error_ = 0.0;
        integral_error_ = 0.0;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg) {
        path_ = *msg;
    }

    // void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    //     // 根据 odom 消息计算 current_pose_
    //     current_pose_.pose.position.x = msg->pose.pose.position.x - originOffsetX;
    //     current_pose_.pose.position.y = msg->pose.pose.position.y - originOffsetY;
    //     current_pose_.pose.position.z = msg->pose.pose.position.z - originOffsetZ;

    //     current_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    //     current_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    //     current_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    //     current_pose_.pose.orientation.w = msg->pose.pose.orientation.w;

    //     current_pose_.header.stamp = msg->header.stamp;
    //     current_pose_.header.frame_id = "map";
        
    //     cout << "current_pose_.pose.position.x = " <<current_pose_.pose.position.x << endl;
    //     cout << "current_pose_.pose.position.y = " <<current_pose_.pose.position.y << endl;
    //     cout << "current_pose_.pose.position.z = " <<current_pose_.pose.position.z << endl;

    //     trackTrajectory();
    // }
    
   
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    
    // if (isFirstFrame) {
    //     // 记录第一帧的位置作为初始位置
        
    //     initial_x = 441485.2023248055;
    //     initial_y = 4423140.423381293;
    //     initial_z = 53.61276412755251;
    //     isFirstFrame = false;  // 后续帧不再是第一帧

    // }

    // 根据当前的坐标减去初始坐标计算相对位置
    current_pose_.pose.position.x = msg->pose.pose.position.x - 441484.81971662195;
    current_pose_.pose.position.y = msg->pose.pose.position.y - 4423140.357555132;
    current_pose_.pose.position.z = msg->pose.pose.position.z - 53.58761507928202;

    // 姿态不变
    current_pose_.pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose_.pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose_.pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose_.pose.orientation.w = msg->pose.pose.orientation.w;

    //odom_yaw
    //current_yaw_odom_ = tf::getYaw(msg->pose.pose.orientation);
    //cout << "current_yaw_odom = " << current_yaw_odom_ << endl;

    // 设置时间戳和坐标系
    current_pose_.header.stamp = msg->header.stamp;
    current_pose_.header.frame_id = "map";

    // 打印当前位置信息
    cout << "current_pose_.pose.position.x = " << current_pose_.pose.position.x << endl;
    cout << "current_pose_.pose.position.y = " << current_pose_.pose.position.y << endl;
    cout << "current_pose_.pose.position.z = " << current_pose_.pose.position.z << endl;
    
    // 轨迹追踪
    trackTrajectory();
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {

        // if (isFirstFrame_2) {
        // // 记录第一帧初始角度
        // initial_yaw = tf::getYaw(msg->orientation);
        // isFirstFrame_2 = false;  // 后续帧不再是第一帧
        // }

        current_yaw_ = tf::getYaw(msg->orientation);
        cout << "current_yaw_ = " << current_yaw_ << endl;
    }
   
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher cmd_pub_;

    nav_msgs::Path path_;
    geometry_msgs::PoseStamped current_pose_;
    double current_yaw_;
    double current_yaw_odom_;

    double lookahead_distance_;
    double kp_, ki_, kd_;
    double last_error_;
    double integral_error_;
    double stop_distance_;
    geometry_msgs::PoseStamped start_point_;

    // double initial_x;
    // double initial_y;
    // double initial_z;
    // double initial_yaw;
    // bool isFirstFrame=true; 
    // bool isFirstFrame_2=true; 

    // std::string base_link_;
    // double originOffsetX_;
    // double originOffsetY_;
    // double originOffsetZ_;

    void trackTrajectory() {
        if (path_.poses.empty()) {
            ROS_WARN("Path is empty, cannot track trajectory.");
            return;
        }

        int closest_idx = -1;
        double min_distance = std::numeric_limits<double>::max();

        start_point_.pose.position.x = path_.poses[0].pose.position.x;
        start_point_.pose.position.y = path_.poses[0].pose.position.y;
        cout<< "start_point_.pose.position.x = " << start_point_.pose.position.x << "start_point_.pose.position.y = " << start_point_.pose.position.y << endl;

        for (size_t i = 0; i < path_.poses.size(); ++i) {
            double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
            double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }

        if (closest_idx == -1) {
            ROS_WARN("Could not find the closest point on the path.");
            return;
        }
        
        // 检查是否接近终点
        geometry_msgs::PoseStamped goal_point = path_.poses.back();
        double goal_dx = goal_point.pose.position.x - current_pose_.pose.position.x;
        double goal_dy = goal_point.pose.position.y - current_pose_.pose.position.y;
        double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);
        cout << "goal_distance = "<<goal_distance<<endl;
        if (goal_distance <= stop_distance_) {
            ROS_INFO("Reached the goal within stop distance. Stopping.");
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_pub_.publish(stop_cmd);
            return;
        }

        geometry_msgs::PoseStamped lookahead_point;
        bool found_lookahead = false;

        for (size_t i = closest_idx; i < path_.poses.size(); ++i) {
            double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
            double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            cout << "path_point" << i << "_x =" << path_.poses[i].pose.position.x << endl;
            cout << "path_point" << i << "_y =" << path_.poses[i].pose.position.y << endl;
            if (distance > lookahead_distance_) {
                if (i > 0) {
                    lookahead_point = path_.poses[i - 1];
                } else {
                    lookahead_point = path_.poses[i];
                }
                found_lookahead = true;
                cout << "lookahead path_point_x = " << path_.poses[i].pose.position.x << endl;
                cout << "lookahead path_point_y = " << path_.poses[i].pose.position.y << endl;
                break;
            }

            if (i == path_.poses.size() - 1) {
                lookahead_point = path_.poses[i];
                found_lookahead = true;
                cout << "lookahead path_point_x = " << path_.poses[i].pose.position.x << endl;
                cout << "lookahead path_point_y = " << path_.poses[i].pose.position.y << endl;
            }
        }

        if (!found_lookahead) {
            ROS_WARN("Could not find a valid lookahead point.");
            return;
        }

        double dx = lookahead_point.pose.position.x - current_pose_.pose.position.x;
        double dy = lookahead_point.pose.position.y - current_pose_.pose.position.y;
        double target_yaw = std::atan2(dy, dx)*M_PI/180;

        cout << "target_yaw = " << target_yaw << endl;

        double heading_error = target_yaw - current_yaw_;

        while (heading_error > M_PI) heading_error -= 2 * M_PI;
        while (heading_error < -M_PI) heading_error += 2 * M_PI;

        integral_error_ += heading_error;
        double derivative_error = heading_error - last_error_;
        last_error_ = heading_error;

        double angular_velocity = kp_ * heading_error + ki_ * integral_error_ + kd_ * derivative_error;
        if (angular_velocity > 0.5) {
            angular_velocity = 0.5; 
        } else if (angular_velocity < -0.5) {
            angular_velocity = -0.5;
        }

        cout << "angular_velocity = " << angular_velocity << endl;

        double linear_velocity = 0.3;

        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_velocity;
        cmd.angular.z = angular_velocity;
        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char **argv) {
    cout << "start" <<endl;
    ros::init(argc, argv, "trajectory_tracking");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    // nhPrivate.getParam("originOffsetX", originOffsetX);
    // nhPrivate.getParam("originOffsetY", originOffsetY);
    // nhPrivate.getParam("originOffsetZ", originOffsetZ);

    TrajectoryTracking tracker(nh);

    ros::spin();
    return 0;
}
