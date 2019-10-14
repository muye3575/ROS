#!/usr/bin/env python
# coding:utf-8

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position and Catersion Position Example: keyboard
reediting by wang zhiqiang email: wzq1992@stu.haust.edu.cn

该例子实现了使用键盘控制saywer机器人关节和直角坐标系移动。
键盘按键对照表：
        '1': joints[0]+
        'q': joints[0]-
        '2': joints[1]+
        'w': joints[1]-
        '3': joints[2]+
        'e': joints[2]-
        '4': joints[3]+
        'r': joints[3]-
        '5': joints[4]+
        't': joints[4]-
        '6': joints[5]+
        'y': joints[5]-
        '7': joints[6]+
        'u': joints[6]-
        'a': x+
        'z': x-
        's': y+
        'x': y-
        'd': z+
        'c': z-

        '8': gripper close
        'i': gripper open
        '9': gripper calibrate

注：按下一次按键后关节和坐标轴移动的距离时设定好的，不能够实现连续移动。
    坐标轴每次移动距离 distance 默认是0.1m，可以通过更改 +0.1 实现每次移动距离设置。例如将 [limb, "x", +0.1],中0.1更改即可。
    关节移动距离也是同样的，在 bindings 词典中更改 [limb, joints[0], 0.1] 的0.1值。

    移动速比设置可以按下 \ 键，输入速比值即可改变移动速比吗，速比范围（0.0, 1.0]，默认速比为_max_joint_speed_ratio=0.2

"""
import argparse

import rospy

import intera_interface
import intera_external_devices

from intera_interface.settings import CHECK_VERSION

from intera_motion_interface.motion_trajectory import MotionTrajectory
from intera_motion_interface.motion_waypoint import (MotionWaypoint,  MotionWaypointOptions)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped


def map_keyboard(side):
    limb = intera_interface.Limb(side)
    tip_name = 'right_hand'
    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()  #得到所有关节名字，变量为列表

    traj_options = TrajectoryOptions()
    traj_options.interpolation_type = 'JOINT'
    traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)

    _max_joint_speed_ratio = 0.2
    _max_joint_accel       = 0.2
    wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=_max_joint_speed_ratio,
                                     max_joint_accel=_max_joint_accel)
    waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)

    def set_j(limb, joint_name, delta):
        # 关节移动
        traj_options.interpolation_type = 'JOINT'
        traj.set_trajectory_options(trajectory_options = traj_options)

        current_position = limb.joint_ordered_angles() #获取当前关节角度
        # 将需要移动的关节角度增减
        current_position[joints.index(joint_name)] = current_position[joints.index(joint_name)] + delta

        wpt_opts1 = MotionWaypointOptions(max_joint_speed_ratio=_max_joint_speed_ratio,
                                         max_joint_accel=_max_joint_accel)
        waypoint.set_waypoint_options(wpt_opts1)
        waypoint.set_joint_angles(current_position)
        traj.clear_waypoints()
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        limb.exit_control_mode()
        # limb.set_joint_positions(joint_command)
        traj.clear_waypoints()
    def set_l(limb, cartesian_axis ,distance):

        traj_options.interpolation_type = 'CARTESIAN'
        traj.set_trajectory_options(trajectory_options = traj_options)

        endpoint_state = limb.tip_state(tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', tip_name)
            return None
        pose = endpoint_state.pose
        if cartesian_axis == 'x':
            pose.position.x += distance
        elif  cartesian_axis == 'y':
            pose.position.y += distance
        elif  cartesian_axis == 'z':
            pose.position.z += distance
        poseStamped = PoseStamped()
        poseStamped.pose = pose

        joint_angles = limb.joint_ordered_angles()
        wpt_opts2 = MotionWaypointOptions(max_joint_speed_ratio=_max_joint_speed_ratio,
                                         max_joint_accel=_max_joint_accel)
        waypoint.set_waypoint_options(wpt_opts2)
        waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)
        traj.clear_waypoints()
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
        limb.exit_control_mode()
        traj.clear_waypoints()
    def set_g(action):
        if has_gripper:
            print('griper ok. ')
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    bindings = {
        '1': (set_j, [limb, joints[0], 0.1], joints[0]+" increase"),
        'q': (set_j, [limb, joints[0], -0.1], joints[0]+" decrease"),
        '2': (set_j, [limb, joints[1], 0.1], joints[1]+" increase"),
        'w': (set_j, [limb, joints[1], -0.1], joints[1]+" decrease"),
        '3': (set_j, [limb, joints[2], 0.1], joints[2]+" increase"),
        'e': (set_j, [limb, joints[2], -0.1], joints[2]+" decrease"),
        '4': (set_j, [limb, joints[3], 0.1], joints[3]+" increase"),
        'r': (set_j, [limb, joints[3], -0.1], joints[3]+" decrease"),
        '5': (set_j, [limb, joints[4], 0.1], joints[4]+" increase"),
        't': (set_j, [limb, joints[4], -0.1], joints[4]+" decrease"),
        '6': (set_j, [limb, joints[5], 0.1], joints[5]+" increase"),
        'y': (set_j, [limb, joints[5], -0.1], joints[5]+" decrease"),
        '7': (set_j, [limb, joints[6], 0.1], joints[6]+" increase"),
        'u': (set_j, [limb, joints[6], -0.1], joints[6]+" decrease"),
        'a': (set_l, [limb, "x", +0.1], " x" + " increase"),
        'z': (set_l, [limb, "x", -0.1], " x" + " decrease"),
        's': (set_l, [limb, "y", +0.1], " y" + " increase"),
        'x': (set_l, [limb, "y", -0.1], " y" + " decrease"),
        'd': (set_l, [limb, "z", +0.1], " z" + " increase"),
        'c': (set_l, [limb, "z", -0.1], " z" + " decrease"),
    }
    if has_gripper:
        bindings.update({
        '8': (set_g, "close", side+" gripper close"),
        'i': (set_g, "open", side+" gripper open"),
        '9': (set_g, "calibrate", side+" gripper calibrate")
        })
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")

    time_tamp = None
    time_label = None
    while not done and not rospy.is_shutdown():
        c = intera_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c == '\\':
                set_speed = input('请输入设定的速度，按下Enter确认。\n')
                _max_joint_speed_ratio= set_speed

            elif c in bindings:
                cmd = bindings[c]
                if c == '8' or c == 'i' or c == '9':
                    cmd[0](cmd[1])
                    print("command: %s" % (cmd[2],))
                else:
                    #expand binding to something like "set_j(right, 'j0', 0.1)"
                    cmd[0](*cmd[1])
                    print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

            time_tamp = rospy.get_time()

        else:
            if (time_tamp is not None)  :
                if time_tamp != time_label :
                    if (rospy.get_time() - time_tamp) > 1 :
                        print ('当前姿态：')
                        print (limb.tip_state(tip_name).pose)
                        # print (limb.tip_state(tip_name).pose.position)
                        # print (limb.tip_state(tip_name).pose.position.x)
                        # print (limb.tip_state(tip_name).pose.orientation)
                        # print (limb.endpoint_pose())
                        # print (limb.endpoint_pose()['position'])
                        # print (limb.endpoint_pose()['orientation'])

                        print ('当前关节角：')
                        print (limb.joint_angles())
                        print ('\n')
                        time_label = time_tamp
    traj.stop_trajectory()

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """

    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )


    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
