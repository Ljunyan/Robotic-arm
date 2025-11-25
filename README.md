# Robotic-arm

轻量级 ROS Noetic + MoveIt 工程，用于 Probot 机械臂的仿真、控制与示例。

## 项目概览
本仓库包含 Probot 机械臂的描述、Gazebo 仿真场景、MoveIt 规划配置与若干 demo 节点，适合学习 ROS + MoveIt 的入门与仿真实验。

## 主要目录
- src/ — ROS 包源代码（probot_description、probot_gazebo、probot_demo、probot_anno_moveit_config 等）
- build/、build_isolated/ — 构建产物
- devel/、devel_isolated/ — catkin 开发空间
- images/ — 项目图片资源
- frames.gv — 帧关系的 Graphviz 描述
- README.md — 本文件

## 功能亮点
- Probot 机械臂 URDF 与模型描述
- Gazebo 仿真场景与插件
- MoveIt 规划配置与演示
- 示例节点与 launch 文件，便于快速测试

## 环境
- Ubuntu 20.04
- ROS Noetic
- MoveIt for Noetic（若需路径规划与交互）
- Gazebo（若需仿真）

## 快速开始（在本机执行）
1. 打开终端并初始化 ROS 环境：
   ```sh
   source /opt/ros/noetic/setup.bash
   ```
2. 进入工作区并构建（在 Windows 下请在 WSL/Ubuntu 中执行）：
   ```sh
   cd workspace
   catkin_make
   ```
3. 构建完成后加载开发环境：
   ```sh
   source devel/setup.bash
   ```
4. 运行示例（示例 launch 位于各包内）：
   - 启动 Gazebo 仿真（示例）：
     ```sh
     roslaunch probot_gazebo probot_world.launch
     ```
   - 启动 MoveIt 演示：
     ```sh
     roslaunch probot_anno_moveit_config demo.launch
     ```
   - 运行 demo 节点：
     ```sh
     rosrun probot_demo <node_name>
     ```

（请根据 repo 中实际的 launch 和节点名称替换上述命令；可在各包目录下查找 *.launch 与可执行文件）

## 开发与调试
- 推荐使用 VS Code（仓库内含 .vscode 配置）
- 修改后重新运行 catkin_make 或使用 catkin_make --pkg <pkg_name>
- 使用 rostopic、rviz、rqt_graph 等工具进行调试与可视化

## 常见问题
- 如果找不到包或依赖，检查 package.xml 并执行 rosdep install：
  ```sh
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  ```
- 若 Launch 文件或节点名不确定，可在包目录使用 ls 查找或使用 roscd/rosls 辅助定位。

## 贡献与许可证
欢迎提交 issue 和 PR。请在提交时说明复现步骤和测试方法。
请在仓库根目录添加 LICENSE 文件并在此处说明许可证类型（例如 MIT、Apache-2.0 等）。

## 参考
- 仓库内包：probot_description, probot_gazebo, probot_demo, probot_anno_moveit_config
- frames.gv：用于可视化坐标帧关系

