#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PoseStamped  
# 全局位姿目标
target_pose = Pose()
target_pose.orientation.x = 0.707108
target_pose.orientation.y = 0.0
target_pose.orientation.z = 0.0
target_pose.orientation.w = 0.707105
target_pose.position.x = 0.351958
target_pose.position.y = 0.0
target_pose.position.z = 0.25

def main():
    # 初始化ROS节点
    rospy.init_node('mobile_arm_control')

    # 初始化moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # 初始化tf2监听器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 底盘速度发布者
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    

    # 订阅底盘的里程计数据
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, (move_group, tf_buffer))

    # 主循环
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # 发布底盘的速度指令
        twist = Twist()
        twist.linear.x = 0.005  # 设置一定的速度
        #twist.angular.z = 0.01 
        pub.publish(twist)

        # 规划和执行
        move_group.set_max_velocity_scaling_factor(0.8)
        move_group.go(wait=True)
        move_group.stop()  # 停止所有剩余的移动
        move_group.clear_pose_targets()

        rate.sleep()

    # 关闭ROS节点
    moveit_commander.roscpp_shutdown()

def odom_callback(data, args):
    move_group, tf_buffer = args
    try:
        # 等待变换，这里我们假设已经有了从odom到base_link的变换
        transform = tf_buffer.lookup_transform('base_link', 'odom', rospy.Time(0), rospy.Duration(1.0))
        
        # 创建一个PoseStamped对象，用于转换
        target_pose_stamped = PoseStamped()
        target_pose_stamped.pose = target_pose
        target_pose_stamped.header.frame_id = 'odom'  # 设置frame_id为odom
        target_pose_stamped.header.stamp = rospy.Time.now()  # 设置当前时间戳

        # 将目标位姿从odom坐标系转换到base_link坐标系
        target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_stamped, transform)
        
        # 更新机械臂的目标位姿
        move_group.set_pose_target(target_pose_base.pose)  # 注意这里我们取出了pose属性

        # 规划和执行
        move_group.go(wait=True)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr('Error on tf transform: %s', e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass