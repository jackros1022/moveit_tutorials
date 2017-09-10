#!/usr/bin/env python
# coding=utf-8

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
# -*- coding:utf-8 –*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
## END_SUB_TUTORIAL
from std_msgs.msg import String


def move_group_python_interface_tutorial():
    ## BEGIN_TUTORIAL
    ##
    ## Setup
    ## CALL_SUB_TUTORIAL imports
    ##=================================初始化工作========================================
    ## 【1】First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
    ## 【2】  实例化robot，这个是接下来robot所有的接口
    robot = moveit_commander.RobotCommander()

    ## Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
    ## 【2-1】 实例化PlanningScence，这个是接下来PlanningScence所有的接口（暂时未使用）
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left arm.
    ## 【3】 实例化robot其中的joint，作为Group；
    ##        如果Robot是个单臂7-DOF的，那么Robot是否等于Group呢？？？
    group = moveit_commander.MoveGroupCommander("left_arm")

    ## We create this DisplayTrajectory publisher which is used below to publish
    ## trajectories for RVIZ to visualize.
    ##=================================相关准备工作========================================
    ## 【4】 （可选）发布轨迹，展示在rviz中，一般要sleep一段时间等待RVIZ准备一下。
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(10)
    print "============ Starting tutorial "

    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## We can get the name of the reference frame for this robot
    ## 获取规划frame（使用group）
    print "============ Reference frame: %s" % group.get_planning_frame()

    ## We can also print the name of the end-effector link for this group
    ## 获取手抓（使用group）
    print "============ Reference frame: %s" % group.get_end_effector_link()

    ## We can get a list of all the groups in the robot
    ## 获取当前Group（使用robot）
    print "============ Robot Groups:"
    print robot.get_group_names()

    ## Sometimes for debugging it is useful to print the entire state of the robot.
    ## 【5】 当前robot的状态，debug使用。（不明白）
    print "============ Printing robot state"
    print robot.get_current_state()
    print "============"

    ## Planning to a Pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the end-effector
    ##=================================笛卡尔空间（指定末端位置）运动规划========================================
    ## 【6-1】 设置pose_target。
    #           此时的pose_target是end_effector_link到达的坐标位置吗？？？
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 0.7
    pose_target.position.y = -0.05
    pose_target.position.z = 1.1
    group.set_pose_target(pose_target)

    ## Now, we call the planner to compute the plan and visualize it if successful
    ## Note that we are just planning, not asking move_group to actually move the robot
    ## 【6-2】 开始规划（只是规划，实际机器人并不会运动）
    plan1 = group.plan()

    print "============ Waiting while RVIZ displays plan1..."
    rospy.sleep(5)

    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    ## 这里是指使用RVIZ重复运动，只是展示效果？？？
    ## But the group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again).
    ## 【6-3】 展示（发布）规划，显示两次
    print "============ Visualizing plan1"
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);

    print "============ Waiting while plan1 is visualized (again)..."
    rospy.sleep(5)

    ## Moving to a pose goal
    ## ^^^^^^^^^^^^^^^^^^^^^
    ##
    ## Moving to a pose goal is similar to the step above except we now use the go() function.
    ## Note that the pose goal we had set earlier is still active and so the robot will try to move to that goal.
    ## We will not use that function in this tutorial
    # since it is a blocking function
    ## and requires a controller to be active
    ## and report success on execution of a trajectory.
    # Uncomment below line when working with a real robot
    ## 【6-4】 真实运动（是阻塞函数）
    # group.go(wait=True)

    ##=================================关节空间（指定单轴）运动规划========================================
    ## Planning to a joint-space goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ##
    ## Let's set a joint space goal and move towards it.
    ## First, we will clear the pose target we had just set.
    ## 【7-1】首先，清除上一次目标位置（因为没有真实运动，所以要清理目标位置）
    group.clear_pose_targets()

    ## Then, we will get the current set of joint values for the group
    group_variable_values = group.get_current_joint_values()
    print "============ Joint values: ", group_variable_values

    ## Now, let's modify one of the joints, plan to the new joint
    ## space goal and visualize the plan
    ## 【7-2】 设置单轴的角度 （若同时设置多个轴的角度呢？？？）
    group_variable_values[0] = 1.0
    group.set_joint_value_target(group_variable_values)

    ## 【7-3】 运动规划
    plan2 = group.plan()

    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)

    ##=================================笛卡尔空间（waypoints）运动规划========================================
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through.
    ##
    waypoints = []
    ## 【8-1】 第1个点
    # start with the current pose
    waypoints.append(group.get_current_pose().pose)

    # first orient gripper and move forward (+x)
    wpose = geometry_msgs.msg.Pose()
    wpose.orientation.w = 1.0
    wpose.position.x = waypoints[0].position.x + 0.1
    wpose.position.y = waypoints[0].position.y
    wpose.position.z = waypoints[0].position.z
    waypoints.append(copy.deepcopy(wpose))

    ## 【8-2】 第2个点
    # second move down
    wpose.position.z -= 0.10
    waypoints.append(copy.deepcopy(wpose))

    ## 【8-3】 第3个点
    # third move to the side
    wpose.position.y += 0.05
    waypoints.append(copy.deepcopy(wpose))

    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    ## 【8-4】 运动规划
    (plan3, fraction) = group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold

    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(5)

    ## Adding/Removing Objects and Attaching/Detaching Objects
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will define the collision object message
    collision_object = moveit_msgs.msg.CollisionObject()

    ## When finished shut down moveit_commander.
    ## 【9】 关闭 moveit_commander
    moveit_commander.roscpp_shutdown()

    ## END_TUTORIAL

    print "============ STOPPING"


if __name__ == '__main__':
    try:
        move_group_python_interface_tutorial()
    except rospy.ROSInterruptException:
        pass
