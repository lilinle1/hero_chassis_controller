# Hero Chassis Controller (英雄机器人麦克纳姆轮底盘控制器)

* **作者 (Author)**: lilinle1
* **开发环境**: Ubuntu (Legion Y7000P) + Docker (`my_ros_noetic:v1`) + CLion (Docker Toolchain)
* **仓库地址**: `https://github.com/lilinle1/hero_chassis_controller`

## 1. 考核指标完成情况总览

| 需求项 | 状态 | 实现概述 |
| :--- | :---: | :--- |
| **必做 1 & 2** | ✅ | 跑通 `rm_description_for_task` 与 `simple_chassis_controller`，使用 `catkin build` 与 CLion `Attach to Process (gzserver)` 进行断点调试 |
| **选做 1：创建新包** | ✅ | 基于 `rm_template` 构建 `hero_chassis_controller`，完整配置 `CMakeLists.txt`、`package.xml` 与插件导出文件 |
| **选做 2：版本管理** | ✅ | 按功能模块分步提交 Commit，包含分支开发与合并冲突处理记录 |
| **选做 3：代码规范** | ✅ | 严格遵循 ROS C++ Style Guide，通过 `roslint_hero_chassis_controller` 静态检查（0 Error, 0 Warning） |
| **选做 4：四轮速度 PID** | ✅ | 使用 4 个独立的 `control_toolbox::Pid` 控制器，支持 `config/controllers.yaml` 参数配置及 `rqt_reconfigure` / `rqt_plot` 实时调参绘图 |
| **选做 5：逆运动学解算** | ✅ | 订阅 `/cmd_vel` (`geometry_msgs/Twist`)，解算四个麦克纳姆轮期望角速度 |
| **选做 6：正运动学里程计** | ✅ | 根据四轮实际角速度解算底盘实际速度并积分，发布 `/odom` (`nav_msgs/Odometry`) 并广播 `odom -> base_link` TF 变换 |
| **选做 7：全局坐标系控制** | ✅ | 使用 `tf::TransformListener::transformVector` 将全局坐标系（`odom`）下的速度指令转换至底盘坐标系（`base_link`），可在配置文件中切换 `base` / `global` 模式 |
| **选做 8：特色功能** | ✅ | 1. 配置文件可设定底盘线加速度与角加速度限制（斜坡速度规划防打滑）；<br>2. 集成 `teleop_twist_keyboard` 全向键盘控制 |

---

## 2. 运动学模型（O型麦克纳姆轮布局）

设底盘轮距为 $W = 0.4\text{ m}$（参数 `wheel_track`），轴距为 $L = 0.4\text{ m}$（参数 `wheel_base`），几何半和 $k = \frac{W + L}{2} = 0.4\text{ m}$，麦轮半径为 $R = 0.07625\text{ m}$。

### 2.1 逆运动学（底盘期望速度 $\rightarrow$ 四轮期望角速度）
$$\begin{bmatrix} \omega_{lf} \\ \omega_{rf} \\ \omega_{lb} \\ \omega_{rb} \end{bmatrix} = \frac{1}{R} \begin{bmatrix} 1 & -1 & -k \\ 1 & 1 & k \\ 1 & 1 & -k \\ 1 & -1 & k \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

### 2.2 正运动学（四轮实际角速度 $\rightarrow$ 底盘实际速度）
$$\begin{aligned}
v_x &= \frac{R}{4} (\omega_{lf} + \omega_{rf} + \omega_{lb} + \omega_{rb}) \\
v_y &= \frac{R}{4} (-\omega_{lf} + \omega_{rf} + \omega_{lb} - \omega_{rb}) \\
\omega_z &= \frac{R}{4k} (-\omega_{lf} + \omega_{rf} - \omega_{lb} + \omega_{rb})
\end{aligned}$$

---

## 3. 个人调试心得与工程排错记录 (Troubleshooting & Optimization)

在开发与调试过程中，针对仿真特性与工具链进行了以下工程优化：

1. **`rqt_plot` 高频阻塞优化（50Hz 分频发布机制）**：
   控制器 `update()` 运行在 1000Hz 物理引擎周期下，最初直接每周期发布 `Float64MultiArray` 会导致基于 Python 的 `rqt_plot` 绘图界面发生严重延迟与坐标轴停滞。为此在代码中引入了分频计数器 `publish_div_`（每 20 个控制周期即 50Hz 发布一次独立话题 `lf_target_vel` 与 `lf_actual_vel`），使阶跃响应曲线丝滑流畅。
2. **仿真时间重置（Reset Time）导致看门狗锁死问题修复**：
   最初在 `/cmd_vel` 回调函数中使用 `ros::Time::now()` 记录时间戳，当在 Gazebo 中重置时间或跨终端发布指令存在微小钟差时，易导致 `(time - last_cmd_time_).toSec()` 出现负数或异常跳变。后续改为通过布尔标志位 `new_cmd_received_` 在 `update(time, period)` 中统一使用物理引擎传入的 `time` 更新时间戳，并采用绝对值判断 `std::abs(...) > cmd_vel_timeout_`，彻底消除了重置仿真后的锁死现象。
3. **PID 参数整定与斜坡加速度配合**：
   测试过高的比例增益（如 $P=18.0, I=1.5$）在阶跃启动时易触发电机最大力矩饱和（URDF 中 `max_effort="5"`）并引发底盘打滑。最终整定为 $P=1.5, I=0.0, D=0.0$，并配合 `max_linear_acc: 4.5` 的斜坡加速度限制器，实现了无超调、低纹波的平滑加减速跟随。
4. **YAML 格式解析排错**：
   排查了 `rosparam` 对紧凑内联格式 `{p: 1.5, ...}`（大括号后无空格）解析失败导致报 `No p gain specified for pid` 的问题，统一规范为标准多行缩进 YAML 结构。

---

### 3.1 编译与代码规范检查
```bash
cd ~/workspace/hero_ws
catkin build -DCMAKE_BUILD_TYPE=Debug
catkin build hero_chassis_controller --no-deps --make-args roslint_hero_chassis_controller
