/**
 * @file cansend_node.cpp
 * @brief 差速驱动机器人CAN控制节点
 * 
 * 功能描述：
 * 1. 订阅cmd_vel速度指令和odom里程计反馈
 * 2. 实现外环PI控制器，带条件积分抗饱和
 * 3. 通过CAN总线发送SDO指令控制电机转速
 * 4. 支持加速度斜坡限制和看门狗保护
 * 
 * 控制架构：
 * cmd_vel → 外环PI控制器 → 差速映射 → CAN SDO → 电机驱动
 *     ↑                                    ↓
 *   odom ← 低通滤波 ← 里程计反馈 ← 电机编码器
 */

/**
 * @file cansend_node.cpp
 * @brief 差速驱动机器人CAN控制节点
 * 
 * 功能描述：
 * 1. 订阅cmd_vel速度指令和odom里程计反馈
 * 2. 实现外环PI控制器，带条件积分抗饱和
 * 3. 通过CAN总线发送SDO指令控制电机转速
 * 4. 支持加速度斜坡限制和看门狗保护
 * 
 * 控制架构：
 * cmd_vel → 外环PI控制器 → 差速映射 → CAN SDO → 电机驱动
 *     ↑                                    ↓
 *   odom ← 低通滤波 ← 里程计反馈 ← 电机编码器
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <sys/time.h>
#include <fcntl.h>
#include <errno.h>
#include <algorithm>

#define CAN_INTERFACE "can0"  // CAN接口名称

// ===== CAN SDO协议定义 =====
#define NODE_ID        0x01     // 从站NodeID
#define REQ_ID  (0x600 + NODE_ID)   // SDO请求ID
#define RESP_ID (0x580 + NODE_ID)   // SDO回应ID

// SDO命令字（expedited transfer）
#define SDO_READ      0x40      // 读请求（Data_0）
#define SDO_WRITE_2B  0x2B      // 写2字节（S16）
#define SDO_READ_2B   0x4B      // 读回应（2字节有效数据）
#define SDO_WRITE_OK  0x60      // 写确认（无数据）

// 对象字典地址
#define WRITE_INDEX   0x2000    // 写速度命令：Index 0x2000，Sub 01/02（通道1/2）
#define READ_INDEX    0x2103    // 读速度反馈：Index 0x2103，Sub 01/02（通道1/2）

/**
 * @struct Limits
 * @brief 速度和加速度限制结构体
 * 用于限制机器人的最大速度和加速度，确保安全运行
 */
struct Limits {
  double max_vx = 0.6, max_wz = 1.5;      // 最大线速度(m/s)和角速度(rad/s)
  double max_ax = 0.6, max_aw = 2.0;      // 最大线加速度(m/s²)和角加速度(rad/s²)
};

/**
 * @class DiffCanOuterPI
 * @brief 差速驱动机器人外环PI控制器类
 * 
 * 主要功能：
 * - 外环PI控制：基于速度误差进行反馈控制
 * - 条件积分抗饱和：防止积分项累积导致控制不稳定
 * - 加速度斜坡限制：平滑速度变化，减少机械冲击
 * - CAN通信：发送SDO指令控制电机转速
 * - 看门狗保护：超时自动停止，确保安全
 */
class DiffCanOuterPI {
public:
  /**
   * @brief 构造函数：初始化控制器参数和ROS接口
   * @param nh ROS节点句柄
   */
  DiffCanOuterPI(ros::NodeHandle& nh): nh_(nh) {
    // ===== 加载机器人物理参数 =====
    nh_.param("wheel_base", L_, 0.50);        // 轮距(米)：左右轮中心距离
    
    // ===== 加载控制参数 =====
    nh_.param("loop_hz", loop_hz_, 100.0);    // 控制循环频率(Hz)：影响控制精度和响应速度
    nh_.param("watchdog_sec", watchdog_sec_, 0.5);  // 看门狗超时时间(秒)：超时后自动停止

    // ===== 加载PI控制器参数 =====
    nh_.param("kp_linear", Kpv_, 0.5);        // 线速度比例系数：影响响应速度和稳态误差
    nh_.param("ki_linear", Kiv_, 0.2);        // 线速度积分系数：消除稳态误差，但可能引起超调
    nh_.param("kp_angular", Kpw_, 0.7);       // 角速度比例系数
    nh_.param("ki_angular", Kiw_, 0.2);       // 角速度积分系数
    
    // 建议先不用微分项，避免噪声放大
    Kdv_ = 0.0; Kdw_ = 0.0;

    // ===== 加载相位超前补偿参数 =====
    nh_.param("lead_compensation_enable", lead_comp_enable_, true);  // 是否启用相位超前补偿
    nh_.param("lead_time_constant", lead_tau_, 0.5);                // 超前时间常数(秒)：补偿系统延迟
    nh_.param("lead_filter_alpha", lead_alpha_, 0.3);               // 超前滤波器系数(0~1)：平滑补偿输出

    // ===== 加载电机反馈控制参数 =====
    nh_.param("motor_feedback_alpha", motor_feedback_alpha_, 0.3);  // 电机反馈低通滤波系数

    // ===== 设置速度到RPM的映射系数 =====
    // 固定值：1 m/s对应的电机转速(RPM)
    rpm_per_mps_ = 762.71;

    // ===== 初始化ROS订阅器和发布器 =====
    sub_cmd_  = nh_.subscribe("cmd_vel", 10, &DiffCanOuterPI::onCmd, this);   // 速度指令订阅
    pub_motor_feedback_ = nh_.advertise<geometry_msgs::Vector3>("motor_feedback", 10); // 电机反馈发布器

    // ===== 初始化CAN通信 =====
    if (!openCan()) { 
      ROS_FATAL("open CAN failed"); 
      ros::shutdown(); 
      return; 
    }

    // ===== 创建固定频率控制循环定时器 =====
    // 使用定时器确保控制循环的固定频率，提高控制精度
    timer_ = nh_.createTimer(ros::Duration(1.0/loop_hz_), &DiffCanOuterPI::onLoop, this);
  }

  /**
   * @brief 析构函数：清理资源
   */
  ~DiffCanOuterPI(){ 
    if (sock_>=0) close(sock_); 
  }

private:
  // ===== CAN通信相关函数 =====
  
  /**
   * @brief 小端写入16位数据
   * @param p 目标缓冲区
   * @param v 16位值
   */
  static inline void le16(uint8_t *p, int16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  /**
   * @brief 小端读出16位数据（带符号）
   * @param p 源缓冲区
   * @return 16位值
   */
  static inline int16_t rd_le16(const uint8_t *p) {
    return static_cast<int16_t>((int16_t)p[0] | ((int16_t)p[1] << 8));
  }

  /**
   * @brief 带超时等待SDO回应帧
   * @param index 对象字典索引
   * @param sub 子索引
   * @param expect_cmd 期望的命令字
   * @param out 输出帧
   * @param timeout_ms 超时时间(毫秒)
   * @return 成功返回true，失败返回false
   */
  bool waitSDO(uint16_t index, uint8_t sub, uint8_t expect_cmd, 
               struct can_frame &out, int timeout_ms = 50) {
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(sock_, &readset);
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    while (true) {
      fd_set rfds = readset;
      int ret = select(sock_ + 1, &rfds, nullptr, nullptr, &tv);
      if (ret <= 0) return false; // 超时或错误

      struct can_frame f{};
      ssize_t n = read(sock_, &f, sizeof(f));
      if (n != sizeof(f)) continue;

      // 只接受来自RESP_ID的11位标准帧
      if (f.can_id != RESP_ID) continue;

      // 验证命令字 + Index + SubIndex
      uint16_t idx = static_cast<uint16_t>(f.data[1] | (f.data[2] << 8));
      uint8_t  subidx = f.data[3];
      if (f.data[0] == expect_cmd && idx == index && subidx == sub) {
        out = f;
        return true;
      }
    }
  }

  /**
   * @brief 发送SDO读请求
   * @param index 对象字典索引
   * @param sub 子索引
   * @return 成功返回true，失败返回false
   */
  bool sendSDORead(uint16_t index, uint8_t sub) {
    struct can_frame f{};
    f.can_id  = REQ_ID;
    f.can_dlc = 8;
    f.data[0] = SDO_READ;
    f.data[1] = static_cast<uint8_t>(index & 0xFF);       // IndexL
    f.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);// IndexH
    f.data[3] = sub;
    f.data[4] = f.data[5] = f.data[6] = f.data[7] = 0;

    return write(sock_, &f, sizeof(f)) == sizeof(f);
  }

  /**
   * @brief 读取电机转速反馈
   * @param left_rpm 左电机转速输出
   * @param right_rpm 右电机转速输出
   * @return 成功返回true，失败返回false
   */
  bool readMotorSpeed(int16_t &left_rpm, int16_t &right_rpm) {
    // 发送读请求
    if (!sendSDORead(READ_INDEX, 0x01) || !sendSDORead(READ_INDEX, 0x02)) {
      return false;
    }

    // 等待响应
    struct can_frame resp1{}, resp2{};
    bool ok1 = waitSDO(READ_INDEX, 0x01, SDO_READ_2B, resp1, 20);
    bool ok2 = waitSDO(READ_INDEX, 0x02, SDO_READ_2B, resp2, 20);

    if (ok1 && ok2) {
      left_rpm = rd_le16(&resp1.data[4]);
      right_rpm = rd_le16(&resp2.data[4]);
      return true;
    }
    return false;
  }

  /**
   * @brief 打开CAN接口
   * @return 成功返回true，失败返回false
   * 
   * 功能：
   * 1. 创建CAN socket
   * 2. 绑定到指定CAN接口
   * 3. 设置非阻塞模式
   */
  bool openCan(){
    // 创建CAN socket
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      ROS_ERROR("Failed to create CAN socket: %s", strerror(errno));
      return false;
    }
    
    // 获取CAN接口索引
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ-1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      ROS_ERROR("Failed to get CAN interface index: %s", strerror(errno));
      close(sock_);
      sock_ = -1;
      return false;
    }
    
    // 绑定socket到CAN接口
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
      ROS_ERROR("Failed to bind CAN socket: %s", strerror(errno));
      close(sock_);
      sock_ = -1;
      return false;
    }
    
    // 设置非阻塞模式：避免CAN通信阻塞控制循环
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
    
    ROS_INFO("CAN interface opened successfully");
    return true;
  }

  /**
   * @brief 发送SDO指令控制电机转速
   * @param subidx 子索引：0x01=左电机，0x02=右电机
   * @param rpm 目标转速(RPM)：16位有符号整数
   * 
   * CAN帧格式：
   * - ID: 0x601 (驱动器地址)
   * - DLC: 8字节
   * - Data[0-3]: SDO命令头 (0x23, 0x00, 0x20, subidx)
   * - Data[4-5]: 转速值 (低字节在前，高字节在后)
   * - Data[6-7]: 填充字节 (0x00)
   */
  void sendSDO(uint8_t subidx, int16_t rpm){
    struct can_frame f{};
    f.can_id  = 0x601;  // 驱动器CAN地址
    f.can_dlc = 8;      // 数据长度8字节
    f.data[0] = 0x23; f.data[1] = 0x00; f.data[2] = 0x20; f.data[3] = subidx; // SDO写命令：0x2000:subidx
    f.data[4] = rpm & 0xFF;                    // 转速低字节
    f.data[5] = (rpm >> 8) & 0xFF;             // 转速高字节
    f.data[6] = 0x00;                          // 填充字节
    f.data[7] = 0x00;                          // 填充字节
    
    // 发送CAN帧，失败时输出警告但不中断控制循环
    if (write(sock_, &f, sizeof(f)) != sizeof(f)) {
      ROS_WARN_THROTTLE(1.0,"CAN write fail");
    }
  }

  // ===== ROS回调函数 =====
  
  /**
   * @brief 速度指令回调函数
   * @param m 速度指令消息
   * 
   * 功能：
   * 1. 记录收到指令的时间(用于看门狗)
   * 2. 对速度指令进行限幅处理
   * 3. 更新目标速度
   */
  // 该回调函数通过ROS订阅geometry_msgs::Twist类型的消息（通常来自/cmd_vel话题），
  // 参数m即为收到的速度指令消息指针。订阅器sub_cmd_在构造函数中通过
  // sub_cmd_ = nh_.subscribe("cmd_vel", 1, &DiffCanOuterPI::onCmd, this);
  // 进行绑定。当有新的/cmd_vel消息发布时，ROS会自动调用onCmd函数，并将消息内容传递给参数m。
  void onCmd(const geometry_msgs::Twist::ConstPtr& m){
    last_cmd_time_ = ros::Time::now(); // 记录收到指令的时间(用于看门狗超时保护)
    target_v_ = clamp(m->linear.x,  -lim_.max_vx, lim_.max_vx);   // 线速度指令限幅
    target_w_ = clamp(m->angular.z, -lim_.max_wz, lim_.max_wz);   // 角速度指令限幅
  }



  // ===== 控制循环 =====
  
  /**
   * @brief 控制主循环回调函数
   * @param ev 定时器事件，包含时间信息
   * 
   * 控制流程：
   * 1. 时间检查和初始化等待
   * 2. 看门狗保护
   * 3. 误差计算
   * 4. PI控制计算
   * 5. 加速度斜坡限制
   * 6. 速度限幅
   * 7. 条件积分更新
   * 8. 差速映射
   * 9. CAN指令发送
   */
  void onLoop(const ros::TimerEvent& ev){
    // ===== 时间计算 =====
    // 使用理想周期计算dt，保证控制器的积分和斜坡计算基于固定周期
    // 即使系统调度有抖动，控制计算依然稳定
    const double dt = (ev.current_expected - ev.last_expected).toSec();
    
    // 若dt非正，直接返回(防止异常情况导致积分发散等问题)
    if (dt <= 0) return;

    // ===== 等待电机反馈初始化完成 =====
    // 确保有有效的电机反馈数据才开始控制
    if (!motor_feedback_initialized_) {
      ROS_DEBUG_THROTTLE(1.0, "Waiting for motor feedback initialization...");
      return;
    }

    // ===== 看门狗保护 =====
    // 如果超过指定时间没有收到速度指令，自动停止机器人
    if ((ros::Time::now() - last_cmd_time_).toSec() > watchdog_sec_) {
      target_v_ = 0.0; target_w_ = 0.0;
    }

    // ===== 相位超前补偿 =====
    // 补偿系统延迟，提高响应速度
    double compensated_target_v = leadCompensation(target_v_, dt);
    double compensated_target_w = leadCompensationAngular(target_w_, dt);

    // ===== 误差计算 =====
    // 计算补偿后的目标速度与电机反馈速度的误差
    double motor_v = (motor_v_left_ + motor_v_right_) / 2.0;  // 电机反馈线速度
    double motor_w = (motor_v_right_ - motor_v_left_) / L_;   // 电机反馈角速度
    const double ev_v = compensated_target_v - motor_v;       // 线速度误差
    const double ev_w = compensated_target_w - motor_w;       // 角速度误差

    // ===== PI控制器计算 =====
    // 控制律：u(t) = Kp*e(t) + Ki*∫e(t)dt
    // 这里采用前馈+反馈的形式：u(t) = r(t) + Kp*e(t) + Ki*∫e(t)dt
    // 其中r(t)是目标值(前馈)，Kp*e(t)是比例项，Ki*∫e(t)dt是积分项
    double v_out = compensated_target_v + Kpv_*ev_v + I_v_;  // 线速度控制输出
    double w_out = compensated_target_w + Kpw_*ev_w + I_w_;  // 角速度控制输出

    // ===== 加速度斜坡限制 =====
    // 限制速度变化率，避免突然的速度变化对机械系统造成冲击
    v_out = ramp(v_out, prev_v_out_, lim_.max_ax*dt);  // 线速度斜坡限制
    w_out = ramp(w_out, prev_w_out_, lim_.max_aw*dt);  // 角速度斜坡限制

    // ===== 速度限幅 =====
    // 确保控制输出不超过机器人的物理限制
    const double v_out_sat = clamp(v_out, -lim_.max_vx, lim_.max_vx);  // 线速度限幅
    const double w_out_sat = clamp(w_out, -lim_.max_wz, lim_.max_wz);  // 角速度限幅

    // ===== 条件积分抗饱和 =====
    // 防止积分项持续累积导致"积分饱和"，提升控制器稳定性
    // 
    // 条件积分逻辑：
    // 1. 如果输出未饱和，正常积分
    // 2. 如果输出饱和，只有在积分有助于"脱离饱和"时才积分：
    //    - 正向饱和但误差为负：积分会减小输出，有助于脱饱和
    //    - 负向饱和但误差为正：积分会增大输出，有助于脱饱和
    
    // 使用阈值比较判断是否饱和(避免浮点数精度问题)
    const double saturation_threshold = 1e-6;  // 饱和判断阈值
    bool v_saturated = std::abs(v_out - v_out_sat) > saturation_threshold;
    bool w_saturated = std::abs(w_out - w_out_sat) > saturation_threshold;
    
    // 判断是否应该积分
    bool should_integrate_v = 
        !v_saturated ||                         // 1. 未饱和时，允许积分
        ((v_out > 0) && (ev_v < 0)) ||         // 2. 正向饱和但误差为负，允许积分
        ((v_out < 0) && (ev_v > 0));           // 3. 负向饱和但误差为正，允许积分

    bool should_integrate_w = 
        !w_saturated ||                         // 1. 未饱和时，允许积分
        ((w_out > 0) && (ev_w < 0)) ||         // 2. 正向饱和但误差为负，允许积分
        ((w_out < 0) && (ev_w > 0));           // 3. 负向饱和但误差为正，允许积分

    // 更新积分项
    if (should_integrate_v) I_v_ += Kiv_ * ev_v * dt; // 线速度积分项更新
    if (should_integrate_w) I_w_ += Kiw_ * ev_w * dt; // 角速度积分项更新

    // ===== 保存当前输出用于下次斜坡计算 =====
    prev_v_out_ = v_out_sat;
    prev_w_out_ = w_out_sat;

    // ===== 差速映射 =====
    // 将机器人的线速度和角速度转换为左右轮速度
    // 差速驱动运动学：vL = v - L*w/2, vR = v + L*w/2
    const double vL = v_out_sat - 0.5*L_*w_out_sat;  // 左轮速度
    const double vR = v_out_sat + 0.5*L_*w_out_sat;  // 右轮速度

    // ===== 速度单位转换 =====
    // 将m/s转换为RPM：rpm = v * rpm_per_mps
    const int16_t rpmL = static_cast<int16_t>(vL * rpm_per_mps_);  // 左轮RPM
    const int16_t rpmR = static_cast<int16_t>(vR * rpm_per_mps_);  // 右轮RPM

    // ===== 发送CAN控制指令 =====
    sendSDO(0x01, rpmL);  // 发送左电机转速指令
    sendSDO(0x02, rpmR);  // 发送右电机转速指令

    // ===== 读取电机反馈 =====
    int16_t left_rpm = 0, right_rpm = 0;
    if (readMotorSpeed(left_rpm, right_rpm)) {
      // 转换为m/s
      double v_left_motor = left_rpm / rpm_per_mps_;
      double v_right_motor = right_rpm / rpm_per_mps_;
      
      // 低通滤波
      if (!motor_feedback_initialized_) {
        motor_v_left_ = v_left_motor;
        motor_v_right_ = v_right_motor;
        motor_feedback_initialized_ = true;
        ROS_INFO("Motor feedback initialized: vL=%.3f, vR=%.3f", motor_v_left_, motor_v_right_);
      } else {
        motor_v_left_ = motor_feedback_alpha_ * v_left_motor + (1.0 - motor_feedback_alpha_) * motor_v_left_;
        motor_v_right_ = motor_feedback_alpha_ * v_right_motor + (1.0 - motor_feedback_alpha_) * motor_v_right_;
      }
      
      // 发布电机反馈
      geometry_msgs::Vector3 motor_feedback;
      motor_feedback.x = motor_v_left_;   // 左轮速度
      motor_feedback.y = motor_v_right_;  // 右轮速度
      motor_feedback.z = (motor_v_right_ - motor_v_left_) / L_;  // 角速度
      pub_motor_feedback_.publish(motor_feedback);
    }

    // ===== 调试信息输出 =====
    // 每0.5秒输出一次控制状态信息
    ROS_DEBUG_THROTTLE(0.5, "cmd v=%.2f/%.2f  motor v=%.2f/%.2f  rpmL=%d rpmR=%d",
                       target_v_, target_w_, motor_v, motor_w, rpmL, rpmR);
  }

  // ===== 工具函数 =====
  
  /**
   * @brief 限制函数：将x限制在[lo, hi]区间内
   * @param x 输入值
   * @param lo 下限
   * @param hi 上限
   * @return 限制后的值
   */
  static double clamp(double x, double lo, double hi) { 
    return std::max(lo, std::min(hi, x)); 
  }
  
  /**
   * @brief 斜坡函数：限制target相对于current的变化速率不超过max_delta
   * @param target 目标值
   * @param current 当前值
   * @param max_delta 最大变化量
   * @return 限制变化率后的值
   * 
   * 作用：实现加速度限制，避免突然的速度变化
   */
  static double ramp(double target, double current, double max_delta) {
    if (target > current + max_delta) return current + max_delta;  // 正向限制
    if (target < current - max_delta) return current - max_delta;  // 负向限制
    return target;  // 变化量在允许范围内，直接返回目标值
  }

  /**
   * @brief 相位超前补偿计算
   * @param target 目标速度
   * @param dt 时间间隔
   * @return 补偿后的目标速度
   * 
   * 相位超前补偿原理：
   * 1. 计算目标速度的变化率：d_target/dt
   * 2. 超前补偿：compensated = target + tau * d_target/dt
   * 3. 低通滤波：平滑补偿输出，避免噪声放大
   * 
   * 作用：补偿系统延迟，提高响应速度
   */
  double leadCompensation(double target, double dt) {
    if (!lead_comp_enable_ || dt <= 0) return target;
    
    // 计算目标速度的变化率（一阶差分近似）
    double d_target_dt = (target - prev_target_v_) / dt;
    
    // 相位超前补偿：compensated = target + tau * d_target/dt
    double compensated = target + lead_tau_ * d_target_dt;
    
    // 低通滤波平滑补偿输出
    double filtered_compensated = lead_alpha_ * compensated + (1.0 - lead_alpha_) * prev_compensated_v_;
    
    // 更新历史值
    prev_target_v_ = target;
    prev_compensated_v_ = filtered_compensated;
    
    return filtered_compensated;
  }

  /**
   * @brief 角速度相位超前补偿计算
   * @param target 目标角速度
   * @param dt 时间间隔
   * @return 补偿后的目标角速度
   */
  double leadCompensationAngular(double target, double dt) {
    if (!lead_comp_enable_ || dt <= 0) return target;
    
    // 计算目标角速度的变化率
    double d_target_dt = (target - prev_target_w_) / dt;
    
    // 相位超前补偿
    double compensated = target + lead_tau_ * d_target_dt;
    
    // 低通滤波平滑补偿输出
    double filtered_compensated = lead_alpha_ * compensated + (1.0 - lead_alpha_) * prev_compensated_w_;
    
    // 更新历史值
    prev_target_w_ = target;
    prev_compensated_w_ = filtered_compensated;
    
    return filtered_compensated;
  }

private:
  // ===== ROS相关对象 =====
  ros::NodeHandle nh_;                // 节点句柄：用于参数读取和ROS接口管理
  ros::Subscriber sub_cmd_;           // 速度指令订阅器
  ros::Publisher pub_motor_feedback_; // 电机反馈发布器
  ros::Timer timer_;                  // 控制循环定时器：确保固定频率控制

  // ===== CAN通信相关 =====
  int sock_{-1};                      // CAN通信socket句柄

  // ===== 车辆物理参数与映射参数 =====
  double L_{0.5};                     // 轮距(米)：左右轮中心距离
  double rpm_per_mps_{0.0};           // 速度到RPM的映射系数：1 m/s对应的电机转速

  // ===== 速度与加速度限制、控制频率与看门狗 =====
  Limits lim_;                        // 速度/加速度上限：确保安全运行
  double loop_hz_{100.0};             // 控制循环频率(Hz)：影响控制精度
  double watchdog_sec_{0.5};          // 看门狗超时时间(秒)：超时后自动停止

  // ===== 外环PI控制器参数 =====
  double Kpv_{0.5};                   // 线速度比例系数：影响响应速度
  double Kiv_{0.2};                   // 线速度积分系数：消除稳态误差
  double Kdv_{0.0};                   // 线速度微分系数(未使用)
  double Kpw_{0.7};                   // 角速度比例系数
  double Kiw_{0.2};                   // 角速度积分系数
  double Kdw_{0.0};                   // 角速度微分系数(未使用)
  double I_v_{0.0};                   // 线速度积分项：累积误差
  double I_w_{0.0};                   // 角速度积分项：累积误差

  // ===== 目标速度及时间参数 =====
  double target_v_{0.0};              // 目标线速度：来自cmd_vel
  double target_w_{0.0};              // 目标角速度：来自cmd_vel
  ros::Time last_cmd_time_{0};        // 上次收到指令的时间：用于看门狗

  // ===== 斜坡记忆(用于加速度限制) =====
  double prev_v_out_{0.0};            // 上一周期输出的线速度：用于斜坡计算
  double prev_w_out_{0.0};            // 上一周期输出的角速度：用于斜坡计算

  // ===== 相位超前补偿相关变量 =====
  bool lead_comp_enable_{true};       // 是否启用相位超前补偿
  double lead_tau_{0.5};              // 超前时间常数(秒)：补偿系统延迟
  double lead_alpha_{0.3};            // 超前滤波器系数(0~1)：平滑补偿输出
  double prev_target_v_{0.0};         // 上一周期目标线速度：用于计算变化率
  double prev_target_w_{0.0};         // 上一周期目标角速度：用于计算变化率
  double prev_compensated_v_{0.0};    // 上一周期补偿后的线速度：用于低通滤波
  double prev_compensated_w_{0.0};    // 上一周期补偿后的角速度：用于低通滤波

  // ===== 电机反馈相关变量 =====
  double motor_feedback_alpha_{0.3};  // 电机反馈低通滤波系数
  bool motor_feedback_initialized_{false}; // 电机反馈是否已初始化
  double motor_v_left_{0.0};          // 左电机反馈速度(m/s)
  double motor_v_right_{0.0};         // 右电机反馈速度(m/s)
};

/**
 * @brief 主函数：初始化ROS节点并启动控制器
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc,char** argv){
  // 初始化ROS节点
  ros::init(argc, argv, "cansend_outerpi_node");
  
  // 创建私有节点句柄(用于参数读取)
  ros::NodeHandle nh("~");
  
  // 创建控制器实例
  DiffCanOuterPI node(nh);
  
  // 进入ROS事件循环
  ros::spin();
  
  return 0;
}