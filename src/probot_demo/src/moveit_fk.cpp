// 包含ROS和MoveIt!所需的头文件
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "moveit_fk_demo");
    // 创建异步旋转器，允许ROS在后台处理事件
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建一个MoveGroupInterface对象，用于控制名为"manipulator"的机械臂
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 设置机械臂达到目标位置时的关节容差
    arm.setGoalJointTolerance(0.001);

    // 设置机械臂的最大加速度和速度的缩放因子
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂移动到预设的"home"位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1); // 等待1秒，确保机械臂完成移动

    // 设置机械臂的目标关节位置
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125};
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    // 应用目标关节位置，并指示机械臂移动到这些位置
    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    sleep(1); // 等待1秒，确保机械臂完成移动

    // 再次控制机械臂回到"home"位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1); // 等待1秒，确保机械臂完成移动

    // 关闭ROS节点
    ros::shutdown(); 

    return 0;
}