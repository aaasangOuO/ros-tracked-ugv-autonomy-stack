#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
using namespace std;

int serial_port;
double wheel_base = 0.5;


// 回调函数，用于接收并处理速度指令
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 从Twist消息中提取线速度和角速度
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    double v_left = linear_x - (angular_z * wheel_base / 2.0);
    double v_right = linear_x + (angular_z * wheel_base / 2.0);
    
     

    // 将速度转换为要发送的字符串格式 "!M linear_x angular_z"
    string command = "!M " + std::to_string((int)(v_left*762.7)) + " " + std::to_string((int)(v_right*762.7)) + "\r";
    //string command ="!M 3 -3\r";
    // 通过串口发送指令
    int ret = write(serial_port, command.c_str(), command.size());
    
    if(ret<0)
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    }
    else{
    ROS_INFO("SENDING message to serialport...");
    cout << command << endl;}

}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "serial_velocity_node");
    ros::NodeHandle nh;

    // 打开串口
    serial_port = open("/dev/ttyUSB1", O_RDWR);
    if (serial_port < 0) {
        std::cerr << "Error " << errno << " from open: " << strerror(errno) << std::endl;
        return 1;
    }

    // 配置串口属性
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    // 设置波特率
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // 配置8数据位，无校验位，1停止位
    tty.c_cflag &= ~PARENB; // 清除校验位
    tty.c_cflag &= ~CSTOPB; // 1个停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 8数据位

    // 禁用硬件流控，启用接收器，忽略调制解调器控制线
    tty.c_cflag &= ~CRTSCTS; // 禁用硬件流控
    tty.c_cflag |= CREAD | CLOCAL; // 打开接收者，忽略调制解调器控制线

    // 关闭软件流控，设置为原始输入和输出
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 关闭软件流控
    tty.c_oflag &= ~OPOST; // 原始输出

    // 应用设置
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
        return 1;
    }

    // 订阅速度指令
    ros::Subscriber sub = nh.subscribe("cmd_vel", 10, velocityCallback);

    // 循环等待回调函数的调用
    ros::spin();

    char buf[256];
    int n = read(serial_port, buf, sizeof(buf)-1);
    buf[n] = '\0';
    

    // 关闭串口
    close(serial_port);
    return 0;
}