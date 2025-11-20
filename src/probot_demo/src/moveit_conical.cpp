// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "get_home_orientation");
//     ros::NodeHandle node_handle;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     // Replace "manipulator" with the name of your move group
//     const std::string PLANNING_GROUP = "manipulator";

//     // Load the robot model
//     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//     robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

//     // Construct a `MoveGroupInterface` object for interaction
//     moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

//     // Get the joint model group
//     const robot_state::JointModelGroup* joint_model_group =
//         kinematic_model->getJointModelGroup(PLANNING_GROUP);

//     // Create a RobotState object to keep track of the current and desired robot states
//     moveit::core::RobotStatePtr robot_state = move_group.getCurrentState();

//     // Set the robot state to the "home" named target
//     robot_state->setToDefaultValues(joint_model_group, "home");

//     // Get the pose of the end-effector link in the "home" state
//     const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink());

//     // Extract the rotation (as a quaternion) from the end-effector's pose
//     Eigen::Quaterniond quaternion(end_effector_state.rotation());

//     // Print out the quaternion
//     ROS_INFO_STREAM("Home position quaternion: " << quaternion.x() << ", "
//                       << quaternion.y() << ", " << quaternion.z() << ", " << quaternion.w());

//     ros::shutdown();
//     return 0;
// }
// 包含所需的头文件
#include <math.h> // 数学函数库
#include <ros/ros.h> // ROS功能的基本头文件
#include <moveit/move_group_interface/move_group_interface.h> // MoveIt!的移动组接口
#include <moveit/robot_trajectory/robot_trajectory.h> // 用于生成和处理机器人轨迹的类
#include <geometry_msgs/Pose.h> // ROS消息类型，用于描述一个位置和方向

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv,"moveit_conical");
    // 启动一个异步旋转器，允许ROS在后台处理消息
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建一个MoveGroupInterface对象，用于控制名为"manipulator"的机械臂
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // 获取机械臂末端执行器的名称
    std::string end_effector_link = arm.getEndEffectorLink();

    // 设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    // 当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // 设置位置和姿态的允许误差
    arm.setGoalPositionTolerance(0.001); // 位置误差为1毫米
    arm.setGoalOrientationTolerance(0.01); // 姿态误差为大约0.57度

    // 设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setMaxVelocityScalingFactor(0.8);

    // 控制机械臂移动到预设的"home"位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 设置机器人末端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.707108; // 四元数表示的末端姿态
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.707105;
    target_pose.position.x = 0.351958; // 末端位置
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.207887;

    // 设置末端的目标位置，并指示机械臂移动到这个位置
    arm.setPoseTarget(target_pose);
    arm.move();
    sleep(1);

    // 创建一个路点列表，用于存放螺旋线路径上的点
    std::vector<geometry_msgs::Pose> waypoints;

    // 将初始位姿加入路点列表
    waypoints.push_back(target_pose);

    // 螺旋线参数
    double centerA = target_pose.position.x; // 螺旋线中心点X坐标
    double centerB = target_pose.position.y; // 螺旋线中心点Y坐标
    double initial_height = target_pose.position.z; // 初始高度
    double initial_radius = 0.01; // 初始半径
    double height_increment_per_turn = 0.02; // 每一圈高度增加的量
    double radius_increment_per_turn = 0.01; // 每一圈半径增加的量
    int turns = 5; // 螺旋线的圈数
    double th_increment = 0.1; // 角度的增量

    // 构建螺旋线路径
    for(double th = 0.0; th < turns * 2 * M_PI; th += th_increment)
    {
        double current_height = initial_height + (th / (2 * M_PI)) * height_increment_per_turn;
        double current_radius = initial_radius + (th / (2 * M_PI)) * radius_increment_per_turn;
        target_pose.position.x = centerA + current_radius * cos(th);
        target_pose.position.y = centerB + current_radius * sin(th);
        target_pose.position.z = current_height;
        waypoints.push_back(target_pose);
    }

    // 笛卡尔空间下的路径规划
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 设置跳跃阈值为0，意味着不允许跳跃
    const double eef_step = 0.01; // 末端执行器步进值
    double fraction = 0.0; // 规划得到的路径占目标路径的分数
    int maxtries = 100; // 最大尝试规划次数
    int attempts = 0; // 已经尝试规划次数

    // 尝试进行路径规划，直到成功或尝试次数达到最大
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    // 如果规划成功，则执行路径
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // 执行运动
        arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, attempts);
    }

    // 控制机械臂回到"home"位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 关闭ROS节点
    ros::shutdown(); 
    return 0;
}