#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from math import radians
from copy import deepcopy
import numpy as np
from math import pi, cos, sin
class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_attached_demo', anonymous=True)
        
        # 是否需要使用笛卡尔空间的运动规划
        cartesian = rospy.get_param('~cartesian', True)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)


        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        arm.set_planning_time(10)

        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
        scene.remove_world_object('cone')  # 移除之前的圆锥体


        # 添加圆锥体到场景
        cone_pose = PoseStamped()
        cone_pose.header.frame_id = 'base_link' 
        cone_pose.pose.position.x = 0.2
        cone_pose.pose.position.y = 0.2
        cone_pose.pose.position.z = 0.15  # 调整圆锥体高度
        # 使圆锥体尖端朝下的方向
        cone_pose.pose.orientation.x = 0.0
        cone_pose.pose.orientation.y = 0.0
        cone_pose.pose.orientation.z = 0.0
        cone_pose.pose.orientation.w = 1.0
        scene.add_cone('cone', cone_pose, 0.06, 0.02)  # 修改半径为0.05
        rospy.sleep(2)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
      

        # 更新当前的位姿
        arm.set_start_state_to_current_state()
        
        # 获取当前位姿数据为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
        # 初始化路点列表
        waypoints = []

        # 将初始位姿加入路点列表
        if cartesian:
            waypoints.append(start_pose)
                     
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z -=0.3
        wpose.position.x += 0.2
        wpose.position.y += 0.0
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            arm.set_pose_target(wpose)
            arm.go()
            rospy.sleep(1)

        # 示例用法
        cone_vertex = [0.2,0.2,0.15]  # 圆锥体顶点坐标
        cone_direction = [0.0, 0.0, 1.0]  # 圆锥体方向，假设为 z 轴负方向
        cone_radius = 0.02 # 圆锥体半径
        cone_height = 0.06 # 圆锥体高度
        num_points = 30  # 螺旋线上的点的数量
        turns = 1  # 螺旋线的转数
        # 计算每一步的角度增量
        th_increment = turns * 2 * np.pi / num_points

        for i in range(num_points):
            th = i * th_increment

            # 计算当前点在圆锥体表面的位置
            x = cone_vertex[0] + cone_radius * cos(th)
            y = cone_vertex[1] + cone_radius * sin(th)
            z = cone_vertex[2] +(cone_height / turns) * (th / (2 * np.pi))  # 修正方向

            # 计算当前点在圆锥体表面的法向量
            tangent_vector = np.array([-sin(th), cos(th), -cone_radius / turns])  # 修正方向
            tangent_vector =-tangent_vector 
            # 将法向量归一化
            tangent_vector /= np.linalg.norm(tangent_vector)

            # 计算末端姿态
            pitch = np.arctan2(tangent_vector[2], np.sqrt(tangent_vector[0]**2 + tangent_vector[1]**2))
            yaw = np.arctan2(tangent_vector[1], tangent_vector[0])

            # 创建 Pose 对象
            wpose = Pose()
            wpose.position.x = x
            wpose.position.y = y
            wpose.position.z = z
            wpose.orientation.x = np.sin(pitch / 2) * np.cos(yaw / 2)
            wpose.orientation.y = -np.sin(pitch / 2) * np.sin(yaw / 2)
            wpose.orientation.z = np.cos(pitch / 2) * np.sin(yaw / 2)
            wpose.orientation.w = np.cos(pitch / 2) * np.cos(yaw / 2)

            waypoints.append(wpose)


        waypoints.append(deepcopy(wpose))


        if cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            arm.set_start_state_to_current_state()
	 
		# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses，路点列表
                                        0.01,        # eef_step，终端步进值
                                        0.0,         # jump_threshold，跳跃阈值
                                        True
                                        )        # avoid_collisions，避障规划
		    
		    # 尝试次数累加
                attempts += 1
		    
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
		             
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        #arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveAttachedObjectDemo()