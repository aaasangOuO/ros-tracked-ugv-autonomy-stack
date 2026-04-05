    #include "ros/ros.h"
    #include "ars548_messages/ObjectList.h"
    #include <visualization_msgs/MarkerArray.h>
    #include<cmath>
    #include <std_msgs/Bool.h>
    #include<memory>
    //引入坐标转换头文件
    #include <tf/transform_listener.h>
    #include<geometry_msgs/PointStamped.h>
    #include "obstacle_avoidance/ObstacleArray.h"
    #include "obstacle_avoidance/Obstacle.h"


    //声明全局指针，以便访问。指向main中创建的transformlistener
    std::shared_ptr<tf::TransformListener> tf_listener_ptr;
    //将marker_pub设置为全局变量，以便在回调函数中可以访问
    ros::Publisher marker_pub;
    //添加一个检测到障碍物就停车的发布器
    ros::Publisher stop_pub;
    //添加一个全局发布器发布转换为车体坐标系的障碍物信息
    ros::Publisher obstacle_pub;

    struct obstacle
    {
        float x;            //x坐标，相对雷达，后续需要坐标转换
        float y;
        float z;    
        float vx;           //x方向速度
        float vy;
        float length;       //目标长度
        float width;        //目标宽度
        float probability;  //存在概率
        ros::Time timestamp;//时间戳

    };

    //避障函数
    bool shouldStop(std::vector<obstacle> obstacles){
        for(const auto& obs:obstacles){
            float distance = sqrt(obs.x*obs.x + obs.y*obs.y);
            //如果有障碍:在前方（x > 0）横向偏移不大（|y| < 某阈值,且距离较近（sqrt(x² + y²) < 安全距离）,则停止
            if(obs.x>0 && abs(obs.y)<1 && distance<3){
                return true;
            }
        }
        return false;

    }

    void printObjectlistNums(const ars548_messages::ObjectList::ConstPtr& objectlist){

        ROS_INFO("Callback triggered! objectlist_objects size: %zu", objectlist->objectlist_objects.size());
        //创建存放障碍物信息的vector容器obstacles
        std::vector<obstacle> obstacles;
        //用objectlist中的字段填充obstacle然后加入容器obstacles中
        int tf_fail_count = 0;
        for(const auto& obj:objectlist->objectlist_objects){
            /*
            创建一个 `PointStamped` 类型的消息。
            这个类型不仅包含一个三维点（x, y, z），还包含时间戳和坐标系信息（frame_id）。
            这是 ROS 中进行 **TF 坐标变换** 的标准数据结构。
            */
            geometry_msgs::PointStamped radar_point;
            /*
            设置当前时间戳。
            TF 要求知道你希望转换哪个时刻的坐标，因为坐标系可能随时间变化。
            一般使用 `ros::Time::now()`，也可以用 `objectlist->header.stamp` 保持与雷达同步。
            */
            // 优化：用当前时间，减少TF外推错误
            radar_point.header.stamp = ros::Time::now();
            radar_point.header.frame_id = "ars548_obstacle";
            //取出座标点以便后续转换
            radar_point.point.x = obj.u_position_x;
            radar_point.point.y = obj.u_position_y;
            radar_point.point.z = obj.u_position_z;
            //创建用于接收转换后座标点的容器,车体坐标系
            geometry_msgs::PointStamped base_point;
            try
            {
                //使用 tf_listener_ptr（tf::TransformListener 的指针）对一个带有坐标系信息的点 radar_point 进行坐标转换
                //将它从其原始坐标系 变换到 "base_link" 坐标系中
                //第一个参数为目标坐标系，第二个参数为原始点，第三个参数为结果
                tf_listener_ptr->transformPoint("base_link",radar_point,base_point);
            }
            catch(tf::TransformException& ex)//如果变换失败（例如 TF 坐标未发布、时间未对齐等），会抛出 tf::TransformException,捕捉异常并防止崩溃
            {
                tf_fail_count++;
                ROS_WARN("TF transform failed for object (%.2f, %.2f, %.2f): %s", radar_point.point.x, radar_point.point.y, radar_point.point.z, ex.what());
                // 失败时用原始点填充，便于调试
                base_point.point = radar_point.point;
            }
            
            //障碍物现在处于车体坐标系中
            obstacle obstacle{
                base_point.point.x,
                base_point.point.y,
                base_point.point.z,
                obj.f_dynamics_absvel_x,
                obj.f_dynamics_absvel_y,
                obj.u_shape_length_edge_mean,
                obj.u_shape_width_edge_mean,
                obj.u_existence_probability,
                ros::Time::now()
            };
            obstacles.push_back(obstacle);
            //打印信息用于调试
            ROS_INFO("Obstacle: (%.3f, %.3f, %.3f), prob: %.3f", obstacle.x, obstacle.y, obstacle.z, obstacle.probability);
        }
        ROS_INFO("Total obstacles: %zu, TF failed: %d", obstacles.size(), tf_fail_count);

        //为每一个障碍物obstacle 构造markerarray，实现可视化
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        for(const auto& obstacle:obstacles){
            visualization_msgs::Marker marker;
            if(obstacle.probability>0.5){
                //"ars548_obstacle" 是雷达的坐标系
                marker.header.frame_id = "base_link";
                marker.header.stamp = ros::Time::now();
                marker.ns = "obstacle_markers";     //设置命名空间，防止污染
                marker.id = i;                      //每个marker唯一
                marker.type = visualization_msgs::Marker::CUBE; //设置类型为长方体
                marker.action = visualization_msgs::Marker::ADD;//添加这个marker到显示中
                marker.pose.position.x = obstacle.x;
                marker.pose.position.y = obstacle.y;
                marker.pose.position.z = obstacle.z;
                marker.scale.x = obstacle.length;
                marker.scale.y = obstacle.width;
                marker.scale.z = 1;     //高度信息位置，暂设定为1m   
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8f;
                marker.lifetime = ros::Duration(0.1);//marker存活时间，Marker 将在 t 秒后自动从 RViz 中消失
                //添加marker到marker_array列表
                marker_array.markers.push_back(marker);
                ++i;
            }
        }
        marker_pub.publish(marker_array);

        //危险障碍物检测与停车判断
        std_msgs::Bool stop_msg;
        //msg是个结构体，用data访问数据
        stop_msg.data = shouldStop(obstacles);
        stop_pub.publish(stop_msg);

        //std::vector<obstacle>自定义的一个 结构体数组，它只是 C++ 内存中的数据，不是 ROS 的标准消息，无法直接跨进程发布和订阅。
        //构造ObstacleArray进行发布
        //使用自己定义的消息类型obstaclearray
        obstacle_avoidance::ObstacleArray obstacle_array_msg;
        obstacle_array_msg.header.stamp = objectlist->header.stamp;
        obstacle_array_msg.header.frame_id = "base_link";
        //将障碍物信息写入
        for(const auto& obs:obstacles){
            obstacle_avoidance::Obstacle obs_msg;
            obs_msg.x = obs.x;
            obs_msg.y = obs.y;
            obs_msg.z = obs.z;
            obs_msg.vx = obs.vx;
            obs_msg.vy = obs.vy;
            obs_msg.length = obs.length;
            obs_msg.width = obs.width;
            obs_msg.probability = obs.probability;
            obs_msg.timestamp = obs.timestamp;
            obstacle_array_msg.Obstacles.push_back(obs_msg);
        }
        obstacle_pub.publish(obstacle_array_msg);
    }


    int main(int argc,char **argv){
        ros::init(argc,argv,"obstacle_node");
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/ars548/ObjectList",10,printObjectlistNums); // 确保话题名大小写一致
        //在堆上创建一个 TransformListener 对象，并让全局指针 tf_listener_ptr 指向它
        //transformlistener持续监听 ROS 中发布的坐标变换，并能把一个点从一个坐标系转换到另一个坐标系
        // 增大TF缓存时间
        tf_listener_ptr = std::make_shared<tf::TransformListener>(ros::Duration(10.0)); 
        //添加makerarray发布者，车体坐标系  这里起到初始化发布器的作用
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles_marker",1);
        stop_pub = nh.advertise<std_msgs::Bool>("should_stop",1);
        //发布障碍物（车体坐标系）
        obstacle_pub = nh.advertise<obstacle_avoidance::ObstacleArray>("obstacle_array", 1);
        ROS_INFO("obstacle_node started, waiting for /radar/ObjectList ...");
        ros::spin();
        return 0;
    }
