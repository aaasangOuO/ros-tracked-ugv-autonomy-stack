#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ars548_messages/ObjectList.h>
#include <ars548_messages/Object.h>
#include <vector>
#include <cmath>

struct SimplePoint {
    float x, y, z, vx, vy;
};

float euclideanDistance(const SimplePoint& a, const SimplePoint& b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

ros::Publisher objectlist_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::vector<SimplePoint> points;
    // 只用x, y, z, v（用f_rangerate填充v），vx、vy用0
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z"), iter_v(*msg, "v");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_v) {
        SimplePoint p{*iter_x, *iter_y, *iter_z, *iter_v, 0}; // vx=速度，vy=0
        points.push_back(p);
    }

    // 简单聚类：每个点与已有聚类中心比较，距离小于阈值就归为一类，否则新建一类
    std::vector<std::vector<SimplePoint>> clusters;
    float cluster_tolerance = 2.0; // 距离阈值
    for (const auto& pt : points) {
        bool assigned = false;
        for (auto& cluster : clusters) {
            if (!cluster.empty() && euclideanDistance(pt, cluster.front()) < cluster_tolerance) {
                cluster.push_back(pt);
                assigned = true;
                break;
            }
        }
        if (!assigned) {
            clusters.push_back({pt});
        }
    }

    // 生成ObjectList
    ars548_messages::ObjectList object_list;
    object_list.header = msg->header;
    object_list.objectlist_objects.clear();
    int id = 0;
    for (const auto& cluster : clusters) {
        if (cluster.size() < 2) continue; // 过滤掉孤立点
        float x_sum=0, y_sum=0, z_sum=0, vx_sum=0, vy_sum=0;
        for (const auto& pt : cluster) {
            x_sum += pt.x; y_sum += pt.y; z_sum += pt.z;
            vx_sum += pt.vx; vy_sum += pt.vy;
        }
        int n = cluster.size();
        ars548_messages::Object obj;
        obj.u_id = id++;
        obj.u_position_x = x_sum / n;
        obj.u_position_y = y_sum / n;
        obj.u_position_z = z_sum / n;
        obj.f_dynamics_absvel_x = vx_sum / n;
        obj.f_dynamics_absvel_y = vy_sum / n;
        obj.u_existence_probability = 1.0;
        object_list.objectlist_objects.push_back(obj);
    }
    object_list.objectlist_numofobjects = object_list.objectlist_objects.size();
    objectlist_pub.publish(object_list);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_to_objectlist_node");
    ros::NodeHandle nh;
    objectlist_pub = nh.advertise<ars548_messages::ObjectList>("/ars548/ObjectList", 10);
    // 订阅/radar/PointCloudDetection
    ros::Subscriber sub = nh.subscribe("/radar/PointCloudDetection", 10, pointCloudCallback);
    ros::spin();
    return 0;
}