# hero_chassis_controller

英雄机器人麦克纳姆轮底盘控制器（RoboMaster 梯度考核项目）。基于 `ros_control` 框架与 `rm_template` 模板开发，实现了麦克纳姆轮正逆运动学解算、四轮独立速度闭环 PID 控制、里程计（Odometry）航位推算与 TF 广播、基于 TF 的全局坐标系速度控制模式切换，以及底盘加速度平滑限制与键盘遥控功能。

## 1. 功能特性与考核需求对应

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

设底盘轮距为 $W = \text{wheel\_track} = 0.4\text{ m}$，轴距为 $L = \text{wheel\_base} = 0.4\text{ m}$，几何半和 $k = \frac{W + L}{2} = 0.4\text{ m}$，麦轮半径为 $R = 0.07625\text{ m}$。

### 2.1 逆运动学（底盘期望速度 $\rightarrow$ 四轮期望角速度）
$$\begin{bmatrix} \omega_{lf} \\ \omega_{rf} \\ \omega_{lb} \\ \omega_{rb} \end{bmatrix} = \frac{1}{R} \begin{bmatrix} 1 & -1 & -k \\ 1 & 1 & k \\ 1 & 1 & -k \\ 1 & -1 & k \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

### 2.2 正运动学（四轮实际角速度 $\rightarrow$ 底盘实际速度）
$$\begin{aligned}
v_x &= \frac{R}{4} (\omega_{lf} + \omega_{rf} + \omega_{lb} + \omega_{rb}) \\
v_y &= \frac{R}{4} (-\omega_{lf} + \omega_{rf} + \omega_{lb} - \omega_{rb}) \\
\omega_z &= \frac{R}{4k} (-\omega_{lf} + \omega_{rf} - \omega_{lb} + \omega_{rb})
\end{aligned}$$

---

## 3. 编译与运行指南

### 3.1 编译与代码规范检查
```bash
cd ~/workspace/hero_ws
catkin build -DCMAKE_BUILD_TYPE=Debug
catkin build hero_chassis_controller --no-deps --make-args roslint_hero_chassis_controller
