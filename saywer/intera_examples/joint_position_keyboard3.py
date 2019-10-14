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
SDK Joint Position Example: keyboard
该段代码和joint_position_keyboard2.py实现的目的一样，
不同点在于当我们运行的代码中使用到 intera_motion_interface.motion_trajector后，
该段代码键盘控制机器人就会失效。
主要原因可能是使用MotionTrajextory类后，rosmaster发布消息会有变化。
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

    joints = limb.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)
    def set_l(limb, cartesian_axis ,distance):
        # traj_options = TrajectoryOptions()
        # traj_options.interpolation_type = 'CARTESIAN'
        # traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
        # wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.2,
        #                                 max_joint_accel=0.2)
        # waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)


        endpoint_state = limb.tip_state(tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', tip_name)
            return None
        pose = endpoint_state.pose
        # print ('*********')
        # print (pose.position)
        if cartesian_axis == 'x':
            pose.position.x += distance
        elif  cartesian_axis == 'y':
            pose.position.y += distance
        elif  cartesian_axis == 'z':
            pose.position.z += distance
        # poseStamped = PoseStamped()
        # poseStamped.pose = pose
        # print (pose.position)
        print ('*********')
        joints_command = limb.ik_request(pose)
        limb.set_joint_positions(joints_command)

        print (limb.has_collided())
        print ("*****")
        # print ("*****")
        # joint_angles = limb.joint_ordered_angles()
        # waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)
        # traj.append_waypoint(waypoint.to_msg())

        # result = traj.send_trajectory()
        # if result is None:
        #     rospy.logerr('Trajectory FAILED to send')
        #
        # if result.result:
        #     rospy.loginfo('Motion controller successfully finished the trajectory!')
        # else:
        #     rospy.logerr('Motion controller failed to complete the trajectory with error %s',
        #                  result.errorId)
        # traj_options.interpolation_type = 'JOINT'
        # joint_angles = limb.joint_ordered_angles()
        # waypoint.set_cartesian_pose(poseStamped, tip_name, joint_angles)
        # traj.append_waypoint(waypoint.to_msg())
        # traj.send_trajectory()
    def set_g(action):
        if has_gripper:
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
        'a': (set_l, [limb, "x", +0.01], " x" + " increase"),
        'z': (set_l, [limb, "x", -0.01], " x" + " decrease"),
        's': (set_l, [limb, "y", +0.01], " y" + " increase"),
        'x': (set_l, [limb, "y", -0.01], " y" + " decrease"),
        'd': (set_l, [limb, "z", +0.01], " z" + " increase"),
        'c': (set_l, [limb, "z", -0.01], " z" + " decrease"),
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
                limb.set_joint_position_speed(set_speed)
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
                        # print (limb.tip_state(tip_name).pose.quaternion)
                        # print (limb.endpoint_pose())
                        # print (limb.endpoint_pose()['position'])
                        # print (limb.endpoint_pose()['orientation'])

                        print ('当前关节角：')
                        print (limb.joint_angles())
                        print ('\n')
                        time_label = time_tamp

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
    # if rs is not None:
    #     rs.stop()
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
