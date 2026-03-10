# UAV 实验执行清单（Checklist）

适用范围：`Pixhawk 6C / Holybro Kakute H7 + PX4 + Vicon + ROS + MAVROS + se3_controller`  
目标：按步骤完成装机、配置、联调与安全起飞。

---

## 1. 装机与接线

### 1.1 飞控与扩展板连线（Pixhawk 6C）
- [ ] `POWER1 -> 扩展板 PWR1`
- [ ] `FMU PWM OUT -> 扩展板 FMU-PWM-in`
- [ ] 电机经电调后接入扩展板

### 1.2 接收机与飞控连线（SBUS）
- [ ] 接收机 `GND/VCC/SBUS` 分别接飞控 `GND/5V/R6`
- [ ] jumper 接收机三焊盘 `GND/VCC/SBUS` 接线正确（黑/红/白）
- [ ] 通电后接收机与遥控器绑定正常

### 1.3 机体安全检查
- [ ] 螺旋桨方向与电机转向匹配
- [ ] 机架固定可靠，无松动件
- [ ] 机载 NUC 安装牢固并已与飞控连接

---

## 2. 飞控固件与基础配置（Holybro Kakute H7）

### 2.1 PX4 源码与 bootloader 编译
- [ ] 拉取 PX4 源码
```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive
make holybro_kakuteh7_bootloader
```
- [ ] 若报 `arm-none-eabi-gcc: Command not found`，已安装：
```bash
sudo apt-get install gcc-arm-none-eabi -y
```
- [ ] 若报 `ModuleNotFoundError`，按提示补齐 Python 依赖（如 `kconfiglib/symforce/lxml/pyserial`）

### 2.2 DFU 模式烧写 bootloader
- [ ] 按住飞控按钮接电脑进入 DFU 模式
- [ ] 执行擦除并写入：
```bash
dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D build/holybro_kakuteh7_bootloader/holybro_kakuteh7_bootloader.bin
```
- [ ] 重新进入 DFU 后再次写入：
```bash
dfu-util -a 0 --dfuse-address 0x08000000 -D build/holybro_kakuteh7_bootloader/holybro_kakuteh7_bootloader.bin
```
- [ ] 日志出现 `File downloaded successfully`

### 2.3 烧写 PX4 固件
- [ ] 断开并重新连接飞控（此时不要按键进入 DFU）
- [ ] 执行：
```bash
make holybro_kakuteh7_default upload
```
- [ ] 若停在 `Waiting for bootloader...`，已检查并修复上方隐藏报错后重试
- [ ] 最终出现 `[100%] Built target upload`

---

## 3. 机载计算机环境（NUC）

### 3.1 ROS/MAVROS 与工作空间
- [ ] 已安装 ROS Noetic + MAVROS
```bash
sudo apt-get install ros-noetic-mavros*
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:HITSZ-MAS/mavros_vicon_bridge.git
git clone git@github.com:HITSZ-MAS/se3_controller.git
```
- [ ] 编译与环境加载成功
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
source ./devel/setup.bash
```

### 3.2 se3_controller 订阅话题确认
- [ ] `se3_controller/src/se3_example.cpp` 里程计订阅已按场景设置
- [ ] 仿真：`/mavros/local_position/odom`
- [ ] 实机：`/mavros/odometry/in`
- [ ] 未使用的话题已注释，避免重复订阅

---

## 4. QGC 与 Vicon 参数配置

### 4.1 QGroundControl 飞控参数
- [ ] 搜索 `CHK`，安全检查相关项已按实验要求设置（教程中为 `disable`）
- [ ] `EKF2_AID_MASK = 24`（仅 vision position + vision yaw fusion）
- [ ] `EKF2_HGT_MODE = Vision`

### 4.2 Vicon 环境
- [ ] Vicon 系统已启动
- [ ] 已创建无人机刚体对象，且机头对准 Vicon 坐标系 `x` 正方向
- [ ] `mavros_vicon_bridge.launch` 中 `datastream_hostport` 为当前 Vicon 主机 IP
- [ ] `subject_name/segment_name` 与 Vicon 对象名一致

### 4.3 遥控器飞行模式
- [ ] 已在 QGC 飞行模式页配置 `Position / Offboard / Land`
- [ ] `Channel 5` 切换逻辑已验证

---

## 5. 起飞前联调（必须全部通过）

### 5.1 五窗口联调
- [ ] 窗口1：
```bash
roslaunch vicon_bridge mavros_vicon_bridge.launch
```
- [ ] 窗口2：
```bash
rostopic echo /mavros/odometry/in
```
- [ ] 移动机体时，里程计位姿变化正确
- [ ] 窗口3：
```bash
rostopic echo /mavros/state
```
- [ ] 遥控切到 `Position` 后，`mode` 正确变化
- [ ] 窗口4：
```bash
roslaunch se3_controller px4_example.launch
```
- [ ] 窗口5：
```bash
rostopic echo /mavros/setpoint_raw/attitude
```

### 5.2 三项方向性验证（默认目标 `(0,0,1)`）
- [ ] 机体放在 `x=0,y=0` 且机头朝 `+x`，`thrust` 为正
- [ ] 机体放在 `x=1,y=0` 且机头朝 `+x`，`body_rate.y` 明显小于 0
- [ ] 机体放在 `x=0,y=1` 且机头朝 `+x`，`body_rate.x` 明显大于 0

---

## 6. 起飞与降落流程

### 6.1 起飞前安全
- [ ] 安全网全部拉起，网内无人
- [ ] 机体额外系绳固定（首飞/调参阶段建议保留）
- [ ] 操作员明确“异常立即切 `Land`”职责
- [ ] 飞行高度受控，避免脱离 Vicon 捕捉范围

### 6.2 起飞步骤
- [ ] 启动 Vicon bridge（窗口1）
- [ ] 启动 `se3_controller`（窗口2）
- [ ] 遥控器 `Channel 5` 先置于 `Land`（中位）
- [ ] 解锁飞控
- [ ] 切换到 `Offboard`
- [ ] 观察无人机是否进入期望位置
- [ ] 通过 `desire_px/desire_py/desire_pz/desire_yaw` 做小幅目标调整

### 6.3 降落步骤
- [ ] 遥控器 `Channel 5` 切回 `Land`
- [ ] 无人机落地后上锁停桨
- [ ] 断电并记录本次参数与异常现象

---

## 7. 常见故障快速排查

- [ ] `Waiting for bootloader...` 长时间不动：回看上方日志，优先排查 Python 依赖缺失
- [ ] `pip3: command not found`：安装 `python3-pip`
- [ ] `module 'serial' has no attribute 'Serial'`：卸载冲突包并重装 `pyserial`
- [ ] 无法稳定悬停：优先检查 Vicon 坐标朝向、对象名、EKF2 两项参数和 odom 话题配置
- [ ] Offboard 异常冲高风险：立即切 `Land`，排查 setpoint 与模式切换链路

---

## 8. 记录模板（每次飞行后）

- [ ] 日期 / 场地 / 操作员：
- [ ] 飞控型号与固件版本：
- [ ] Vicon IP 与对象名：
- [ ] 起飞前联调是否 100% 通过：
- [ ] 本次目标轨迹/高度：
- [ ] 异常现象与复现条件：
- [ ] 处理动作与结果：

