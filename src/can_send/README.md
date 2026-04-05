# CAN发送节点 - 专业反馈控制系统

## 功能描述

这是一个专业的差速驱动机器人反馈控制系统，采用外环PI控制器和固定频率控制循环，具有以下高级特性：

- **外环PI控制**: 前馈+反馈的混合控制策略
- **条件积分**: 抗积分饱和的智能积分控制
- **加速度斜坡**: 执行层友好的平滑控制
- **相位超前补偿**: 补偿系统延迟，提高响应速度
- **电机反馈控制**: 直接读取电机转速进行闭环控制
- **低通滤波**: 降低电机反馈噪声
- **看门狗保护**: 超时自动停止
- **固定频率控制**: 100Hz高精度控制循环

## 主要特性

1. **订阅话题**:
   - `cmd_vel` (geometry_msgs/Twist): 速度指令输入

2. **发布话题**:
   - `motor_feedback` (geometry_msgs/Vector3): 电机反馈信息
     - x: 左轮速度 (m/s)
     - y: 右轮速度 (m/s)  
     - z: 角速度 (rad/s)

2. **专业控制算法**:
   - 外环PI控制器（前馈+反馈）
   - 条件积分抗饱和
   - 加速度斜坡限制
   - 相位超前补偿
   - 电机反馈控制
   - 低通滤波降噪
   - 看门狗保护机制

3. **CAN通信**:
   - 基于SDO协议的CAN通信
   - 支持双电机差速控制
   - 32位RPM精度
   - 仅发送控制指令，不处理应答
   - 简化的错误处理

## 控制原理

### 电机反馈控制架构
```
cmd_vel → 相位超前补偿 → PI控制器 → 差速映射 → CAN SDO发送
                                    ↓
电机反馈 ← SDO读取 ← 电机驱动器 ← 电机转速
    ↓
低通滤波 → 误差计算 → 闭环控制
```

### 外环PI控制器
```
输出 = 前馈 + 比例项 + 积分项
v_out = target_v + Kp*ev + I_v
w_out = target_w + Kp*ew + I_w
```

### 条件积分（抗饱和）
- **未饱和时**：正常进行积分
- **饱和时**：只有在有助于脱饱和时才积分
  - 正向饱和（输出>0）但误差<0：积分（减小输出）
  - 负向饱和（输出<0）但误差>0：积分（增大输出）
  - 其他情况：停止积分（防止积分饱和）
- 防止积分饱和导致的超调和振荡

### 加速度斜坡
```
v_out = ramp(v_out, prev_v_out, max_ax*dt)
w_out = ramp(w_out, prev_w_out, max_aw*dt)
```

### 相位超前补偿
```
补偿输出 = 目标值 + τ * d(目标值)/dt
compensated = target + tau * d_target_dt
```

**原理**：
- 计算目标速度的变化率
- 超前补偿：compensated = target + τ * d_target/dt
- 低通滤波平滑补偿输出，避免噪声放大
- 补偿系统延迟，提高响应速度

### 差速映射
```
vL = v_out - 0.5*L*w_out  // 左轮速度
vR = v_out + 0.5*L*w_out  // 右轮速度
```

## 参数配置

### 配置文件 (config/pid_params.yaml)

```yaml
cansend_outerpi_node:
  # 车辆物理参数
  wheel_base: 0.50      # 轮距 (米)
  
  # 控制频率和看门狗
  loop_hz: 100.0        # 控制循环频率 (Hz)
  watchdog_sec: 0.5     # 看门狗超时时间 (秒)
  
  # 速度限制
  max_vx: 0.6           # 最大线速度 (m/s)
  max_wz: 1.5           # 最大角速度 (rad/s)
  max_ax: 0.6           # 最大线加速度 (m/s²)
  max_aw: 2.0           # 最大角加速度 (rad/s²)
  
  # 反馈控制参数
  kp_linear: 0.5        # 线速度比例系数
  ki_linear: 0.2        # 线速度积分系数
  kp_angular: 0.7       # 角速度比例系数
  ki_angular: 0.2       # 角速度积分系数
  
  # 滤波参数
  alpha_odom: 0.3       # 里程计低通滤波系数
  
  # 相位超前补偿参数
  lead_compensation_enable: true   # 是否启用相位超前补偿
  lead_time_constant: 0.5          # 超前时间常数(秒)：补偿系统延迟
  lead_filter_alpha: 0.3           # 超前滤波器系数(0~1)：平滑补偿输出
  
  # 电机反馈参数
  use_motor_feedback: true         # 是否使用电机反馈
  motor_feedback_alpha: 0.3        # 电机反馈低通滤波系数(0~1)
  
  # 转速转换参数 (固定值)
  # rpm_per_mps: 762.71  # 每米/秒对应的RPM (代码中硬编码)
  
**注意**: `rpm_per_mps` 参数现在在代码中硬编码为 762.71，不再从配置文件读取。这个值表示 1 m/s 对应的电机转速(RPM)。
```

## 使用方法

### 1. 编译
```bash
cd ~/catkin_ws4
catkin_make
```

### 2. 启动节点
```bash
roslaunch can_send can_send.launch
```

### 3. 发送速度指令
```bash
# 发布线速度指令
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# 发布角速度指令
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10
```

## 调试建议

### 1. 参数调整顺序
1. **调整速度限制**: 根据机器人实际能力设置
2. **调整加速度限制**: 确保平滑运动
3. **调整相位超前补偿**: 根据系统延迟调整lead_time_constant
4. **调整比例系数**: 先调Kp获得基本响应
5. **调整积分系数**: 再调Ki消除稳态误差
6. **调整滤波系数**: 最后调motor_feedback_alpha平衡响应和噪声

### 2. 监控话题
```bash
# 查看电机反馈
rostopic echo /motor_feedback

# 查看控制输出
rostopic echo /cmd_vel

# 查看节点日志
rosnode info /cansend_outerpi_node
```

### 3. 常见问题解决
- **振荡过大**: 减小Kp或增大motor_feedback_alpha
- **响应过慢**: 增大Kp或减小motor_feedback_alpha，或增大lead_time_constant
- **稳态误差**: 增大Ki
- **超调严重**: 减小Ki或增大加速度限制
- **噪声过大**: 减小motor_feedback_alpha或lead_filter_alpha
- **延迟过大**: 增大lead_time_constant或减小lead_filter_alpha



## 注意事项

1. 确保CAN接口正常工作
2. 检查电机反馈是否正常读取
3. 根据实际机器人调整物理参数
4. 注意速度限制的安全性设置
5. 定期检查看门狗超时设置
6. **CAN通信**：
   - 发送控制指令并读取电机反馈
   - 实现真正的闭环控制
   - 非阻塞CAN通信，避免阻塞控制循环 