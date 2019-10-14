#!/usr/bin/env python
#  coding: utf-8
# author: 王志强 email: wzq1992@stu.haust.edu.cn
# date: 2019-10-9

# 将Saywer机器人关节移动和直角坐标移动做成图形界面
# 插件 wxPython 开发图形界面，
#  例子原型为joint_position_keyboard2.py
''' 
Usage:

1、将该程序复制到目录：~/ros_wx/src/intera_sdk/intera_examples/scripts/
或者在上述目录中新建一个python程序，复制下面代码。
2、改变该代码的读写属性：chmod +x ~/ros_wx/src/intera_sdk/intera_examples/scripts/joint_position_gui.py
3、catkin_make： cd ~/ros_ws && catkin_make 
4、运行该代码： cd ~/ros_ws && ./inters.sh && rosrun intera_examples joint_position_gui.py

'''


import wx
import rospy

import intera_interface
import intera_external_devices

from intera_interface.settings import CHECK_VERSION

from intera_motion_interface.motion_trajectory import MotionTrajectory
from intera_motion_interface.motion_waypoint import MotionWaypoint,  MotionWaypointOptions
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped


WINDOWS_TITLE = u"机器人示教界面"
APP_backgrand_picture = u'/home/robotic/图片/2019-10-10 01-03-04屏幕截图.png' # 背景图片地址
APP_backgrand_picture1 = u'/home/robotic/图片/2019-10-09 18-14-01屏幕截图.png' # 背景图片地址
APP_backgrand_picture2 = u'/home/robotic/图片/2019-10-09 18-14-29屏幕截图.png' # 背景图片地址
APP_picture1 = u'/home/robotic/图片/imag1.jpeg' # 图片地址
APP_picture2 = u'/home/robotic/图片/imag2.jpeg' # 图片地址
APP_picture3 = u'/home/robotic/图片/imag3.jpeg' # 图片地址

APP_icon1 = u'/home/robotic/图片/icon1.ico' # icon图片地址


class mainFrame(wx.Frame):
    '''程序主窗口类，继承自wx.Frame'''

    def __init__(self):

        wx.Frame.__init__(self, None, -1, WINDOWS_TITLE, style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)
        # 默认style是下列项的组合：wx.MINIMIZE_BOX | wx.MAXIMIZE_BOX | wx.RESIZE_BORDER | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN

        # 创建面板
        panel = wx.Panel(self, -1)
        # 利用wxpython的GridBagSizer()进行页面布局
        sizer = wx.GridBagSizer(1, 2)  # 列间隔为1，行间隔为2

        # self.SetBackgroundColour(wx.Colour(224, 224, 0)) #更改窗口背景颜色
        # self.SetSize((860, 640)) #设置窗口大小
        self.Center() # 窗口出现时，使其位于显示器的中央
        # 设置背景图片
        main_image = wx.Image(APP_backgrand_picture, wx.BITMAP_TYPE_ANY)
        main_image = main_image.ConvertToBitmap()
        window_size = main_image.GetWidth(), main_image.GetHeight()
        self.background_image = wx.StaticBitmap(parent=panel, bitmap=main_image)

        self.SetSize(window_size)  #设置窗口大小

        # 以下代码处理图标
        icon = wx.EmptyIcon()
        # # 使用其他图像文件作为图标
        icon.CopyFromBitmap(wx.BitmapFromImage(wx.Image(APP_picture3, wx.BITMAP_TYPE_JPEG).Rescale(60,60)))
        # icon = wx.Icon(APP_icon1, wx.BITMAP_TYPE_ICO)  # 读取图标文件
        self.SetIcon(icon)

        """
         以上文件是设置窗口大小和图标，背景图片或者背景颜色。
         下面三行设置按钮的方法是一样的，形式却不一样。
         wx.Button(panel, 7, "X +")
         wx.Button(parent=panel, id=7, label="X +")
         wx.Button(parent=panel, id=7, label="X +", pos=(100,24))
        """

        self.distance = 0.01
        self.delta = 0.1
        self.tip_name = 'right_hand'
        # 调用saywer机器人的机械臂类，获取机械的消息
        self.limb = intera_interface.Limb("right")
        # 确认夹爪是否安装
        try:
            self.gripper = intera_interface.Gripper('right_gripper') #夹爪类，控制夹爪开合
        except:
            self.has_gripper = False
            rospy.loginfo("The electric gripper is not detected on the robot.")
        else:
            self.has_gripper = True

        self.joints = self.limb.joint_names() #关节名字，以列表输出。
        # 设置轨迹运动的选项
        self.traj_options = TrajectoryOptions()
        self.traj_options.interpolation_type = 'JOINT'
        self.traj = MotionTrajectory(trajectory_options=self.traj_options, limb=self.limb)

        # 设置默认机器人移动速比和移动精度
        self._max_joint_speed_ratio = 0.2
        self._max_joint_accel = 0.1
        self.wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=self._max_joint_speed_ratio,
                                              max_joint_accel=self._max_joint_accel)
        self.waypoint = MotionWaypoint(options=self.wpt_opts.to_msg(), limb=self.limb)

        """
        设置窗体布局，添加按钮和文本框。
           使用静态文本框显示位姿和关节角度;
           使用按键设置实现机器人运动
           使用滑动条调节运动速比和运动精度以及每次运动距离
           
           
        """

        #***************文本框设置**********************

        #显示位姿 直角坐标和四元数
        result_pose = self.limb.tip_state(self.tip_name).pose
        display_pose = u'位置:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" %result_pose.position.x) + '\n'
        display_pose += '\t' + 'y: ' + str("%.4f" %result_pose.position.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" %result_pose.position.z) + '\n'
        display_pose += u'姿态:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" %result_pose.orientation.x) + '\n'
        display_pose += '\t' + 'Y: ' + str("%.4f" %result_pose.orientation.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" %result_pose.orientation.z) + '\n'
        display_pose += '\t' + 'W: ' + str("%.4f" %result_pose.orientation.w) + '\n'

        self.text_pose = wx.StaticText(panel, style=wx.TE_MULTILINE | wx.HSCROLL)
        sizer.Add(self.text_pose, pos=(0, 0), flag=wx.ALL, border=5)
        self.text_pose.SetLabel(display_pose)

        #显示关节角度
        result_joints_angles = self.limb.joint_angles()
        display_angles = ''
        for key, value in result_joints_angles.items():
            display_angles =  key[6:8] + ':  ' + str("%.6f" %value) + '\n' + display_angles
        display_angles = u'关节角度：\n' + display_angles
        self.text_joint_angles = wx.StaticText(panel, style=wx.TE_MULTILINE | wx.HSCROLL)
        sizer.Add(self.text_joint_angles, pos=(0, 1), flag=wx.ALL, border=5)
        self.text_joint_angles.SetLabel(display_angles)

        # 显示按键的label
        self.text1 = wx.StaticText(panel, label="按键label")
        sizer.Add(self.text1, pos=(2, 0), flag=wx.ALL, border=5)


        #****************按钮设置**********************
        #    直角坐标运动
        btn_X_up = wx.Button(panel, -1, "X +")  #定义按钮
        sizer.Add(btn_X_up, pos=(4, 0), flag=wx.ALL, border=5) #布局按钮位置
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_X_up) # 触发按钮

        btn_X_down = wx.Button(panel, -1, "X -")
        sizer.Add(btn_X_down, pos=(4, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_X_down)

        btn_Y_up = wx.Button(panel, -1, "Y +")
        sizer.Add(btn_Y_up, pos=(5, 0), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_Y_up)

        btn_Y_down = wx.Button(panel, -1, "Y -")
        sizer.Add(btn_Y_down, pos=(5, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_Y_down)

        btn_Z_up = wx.Button(panel, -1, "Z +")
        sizer.Add(btn_Z_up, pos=(6, 0), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_Z_up)

        btn_Z_down = wx.Button(panel, -1, "Z -")
        sizer.Add(btn_Z_down, pos=(6, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_line, btn_Z_down)

        #  夹爪按钮
        btn_griper_open = wx.Button(panel, -1, "open")
        sizer.Add(btn_griper_open, pos=(7, 0), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_griper, btn_griper_open)

        btn_griper_close = wx.Button(panel, -1, "close")
        sizer.Add(btn_griper_close, pos=(7, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_griper, btn_griper_close)

        btn_griper_calibration = wx.Button(panel, -1, "calibrate")
        sizer.Add(btn_griper_calibration, pos=(7, 3), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_griper, btn_griper_calibration)

        #   关节运动按钮
        btn_j0_up = wx.Button(panel, -1, "J0 +")
        sizer.Add(btn_j0_up, pos=(1, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j0_up)

        btn_j0_down = wx.Button(panel, -1, "J0 -")
        sizer.Add(btn_j0_down, pos=(1, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j0_down)

        btn_j1_up = wx.Button(panel, -1, "J1 +")
        sizer.Add(btn_j1_up, pos=(2, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j1_up)

        btn_j1_down = wx.Button(panel, -1, "J1 -")
        sizer.Add(btn_j1_down, pos=(2, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j1_down)

        btn_j2_up = wx.Button(panel, -1, "J2 +")
        sizer.Add(btn_j2_up, pos=(3, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j2_up)

        btn_j2_down = wx.Button(panel, -1, "J2 -")
        sizer.Add(btn_j2_down, pos=(3, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j2_down)

        btn_j3_up = wx.Button(panel, -1, "J3 +")
        sizer.Add(btn_j3_up, pos=(4, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j3_up)

        btn_j3_down = wx.Button(panel, -1, "J3 -")
        sizer.Add(btn_j3_down, pos=(4, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j3_down)

        btn_j4_up = wx.Button(panel, -1, "J4 +")
        sizer.Add(btn_j4_up, pos=(5, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j4_up)

        btn_j4_down = wx.Button(panel, -1, "J4 -")
        sizer.Add(btn_j4_down, pos=(5, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j4_down)

        btn_j5_up = wx.Button(panel, -1, "J5 +")
        sizer.Add(btn_j5_up, pos=(6, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j5_up)

        btn_j5_down = wx.Button(panel, -1, "J5 -")
        sizer.Add(btn_j5_down, pos=(6, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j5_down)

        btn_j6_up = wx.Button(panel, -1, "J6 +")
        sizer.Add(btn_j6_up, pos=(7, 4), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j6_up)

        btn_j6_down = wx.Button(panel, -1, "J6 -")
        sizer.Add(btn_j6_down, pos=(7, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_BUTTON, self.move_joint, btn_j6_down)

        #****************滑动条设置***************
        # 滑动条文本
        text2 = wx.StaticText(panel, label=u"移动速比")
        sizer.Add(text2, pos=(8, 0), flag=wx.ALL, border=5)

        slider_bar_speed = wx.Slider(parent=panel, id=-1, value=self._max_joint_speed_ratio*100, minValue=1, maxValue=100, size=(200, 50),
                               style=wx.SL_AUTOTICKS | wx.SL_LABELS , name=u'移动速比')
        # slider_bar_speed.SetTick(1)  # 滑块刻度间隔
        slider_bar_speed.SetPageSize(1) #每次滑块移动距离
        sizer.Add(slider_bar_speed, pos=(8, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_SLIDER, self.set_speed, slider_bar_speed)
        # 滑动条文本
        text3 = wx.StaticText(panel, label=u"精确度")
        sizer.Add(text3, pos=(9, 0), flag=wx.ALL, border=5)

        slider_bar_sccel = wx.Slider(parent=panel, id=-1, value=1, minValue=self._max_joint_accel*100, maxValue=100, size=(200, 50),
                               style=wx.SL_AUTOTICKS | wx.SL_LABELS | wx.SL_TOP, name=u'精确度')
        # slider_bar_sccel.SetTickFreq(1, 1)  # 滑块刻度间隔
        slider_bar_sccel.SetPageSize(1) #每次滑块移动距离
        sizer.Add(slider_bar_sccel, pos=(9, 1), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_SLIDER, self.set_speed, slider_bar_sccel)

        text4 = wx.StaticText(panel, label=u"直角坐标系每次移动距离")
        sizer.Add(text4, pos=(8, 4), flag=wx.ALL, border=5)

        slider_bar_distance = wx.Slider(parent=panel, id=-1, value=self.distance*1000, minValue=0, maxValue=60, size=(200, 50),
                               style=wx.SL_AUTOTICKS | wx.SL_LABELS | wx.SL_TOP, name=u'直角距离')
        # slider_bar_distance.SetTickFreq(1, 1)  # 滑块刻度间隔
        slider_bar_distance.SetPageSize(1) #每次滑块移动距离
        sizer.Add(slider_bar_distance, pos=(8, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_SLIDER, self.set_move_distance, slider_bar_distance)

        text5 = wx.StaticText(panel, label=u"关节坐标每次移动距离")
        sizer.Add(text5, pos=(9, 4), flag=wx.ALL, border=5)

        slider_bar_delta = wx.Slider(parent=panel, id=-1, value=self.delta*100, minValue=0, maxValue=100, size=(200, 50),
                                       style=wx.SL_AUTOTICKS | wx.SL_LABELS | wx.SL_TOP, name=u'关节距离')
        # slider_bar_delta.SetTickFreq(1, 1)  # 滑块刻度间隔
        slider_bar_delta.SetPageSize(1) #每次滑块移动距离
        sizer.Add(slider_bar_delta, pos=(9, 5), flag=wx.ALL, border=5)
        self.Bind(wx.EVT_SLIDER, self.set_move_distance, slider_bar_delta)


        # 将Panmel适应GridBagSizer()放置
        panel.SetSizerAndFit(sizer)


    def set_move_distance(self, event):
        # 定义运动距离滑动条响应
        event_object = event.GetEventObject()
        value = event_object.GetValue()
        name = event_object.GetName()
        if name == u'关节距离':
            self.delta = value/100.0
        elif name == u'直角距离':
            self.distance =  value/1000.0

    def set_speed(self,event):
        #定义运动速比和运动精度响应
        event_object = event.GetEventObject()
        value = event_object.GetValue()/100.0
        name = event_object.GetName()

        if name == u"移动速比" :
            self._max_joint_speed_ratio = value
        elif name ==u'精确度' :
            self._max_joint_accel = value
        wpt_opts1 = MotionWaypointOptions(max_joint_speed_ratio= self._max_joint_speed_ratio,
                                          max_joint_accel=self._max_joint_accel)
        self.waypoint.set_waypoint_options(wpt_opts1)

    def move_joint(self,event):
        # 定义关节运动响应
        event_object = event.GetEventObject()
        label = event_object.GetLabel()
        if label[3] == '-':
            if self.delta >= 0.0:
                self.delta = 0.0-self.delta
        if label[3] == '+':
            if self.delta <= 0.0:
                self.delta = 0.0-self.delta
        self.text1.SetLabel(label+'  '+str(abs(self.delta)))

        joint_name = self.joints[int(label[1])]
        self.traj_options.interpolation_type = 'JOINT'
        self.traj.set_trajectory_options(trajectory_options=self.traj_options)

        current_position = self.limb.joint_ordered_angles()
        current_position[self.joints.index(joint_name)] = current_position[self.joints.index(joint_name)] + self.delta
        self.waypoint.set_joint_angles(current_position)
        self.traj.clear_waypoints() # 清除上一次保存的位姿。
        self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)

        self.limb.exit_control_mode()
        # self.traj.clear_waypoints()

        #显示位姿
        result_pose = self.limb.tip_state(self.tip_name).pose
        display_pose = u'位置:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" % result_pose.position.x) + '\n'
        display_pose += '\t' + 'y: ' + str("%.4f" % result_pose.position.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" % result_pose.position.z) + '\n'
        display_pose += u'姿态:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" % result_pose.orientation.x) + '\n'
        display_pose += '\t' + 'Y: ' + str("%.4f" % result_pose.orientation.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" % result_pose.orientation.z) + '\n'
        display_pose += '\t' + 'W: ' + str("%.4f" % result_pose.orientation.w) + '\n'

        self.text_pose.SetLabel(display_pose)

        #显示关节角度
        result_joints_angles = self.limb.joint_angles()
        display_angles = ''
        for key, value in result_joints_angles.items():
            display_angles =  key[6:8] + ':  ' + str("%.6f" % value) + '\n' + display_angles
        display_angles = u'关节角度：\n' + display_angles
        self.text_joint_angles.SetLabel(display_angles)


    def move_line(self, event):
        # 直角坐标系运动
        event_object = event.GetEventObject()
        label = event_object.GetLabel()
        if label[2] == '-':
            if self.distance >= 0.0:
                self.distance = 0.0-self.distance
        elif label[2] == '+':
            if self.distance <= 0.0:
                self.distance = 0.0-self.distance
        self.text1.SetLabel(label+'  '+str(abs(self.distance)))


        self.traj_options.interpolation_type = 'CARTESIAN'
        self.traj.set_trajectory_options(trajectory_options=self.traj_options)

        endpoint_state = self.limb.tip_state(self.tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', self.tip_name)
            return None
        pose = endpoint_state.pose
        cartesion_axis = label[0].lower()

        if cartesion_axis == 'x':
            pose.position.x += self.distance
        elif cartesion_axis == 'y':
            pose.position.y += self.distance
        elif cartesion_axis == 'z':
            pose.position.z += self.distance

        poseStamped = PoseStamped()
        poseStamped.pose = pose
        joint_angles = self.limb.joint_ordered_angles()

        self.waypoint.set_cartesian_pose(poseStamped, self.tip_name, joint_angles)
        self.traj.clear_waypoints() #清除保留的点
        self.traj.append_waypoint(self.waypoint.to_msg())

        result = self.traj.send_trajectory()
        if result is None:
            rospy.logerr('Trajectory FAILED to send')

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
        self.limb.exit_control_mode()
        # self.traj.clear_waypoints()



        result_pose = self.limb.tip_state(self.tip_name).pose
        display_pose = u'位置:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" % result_pose.position.x) + '\n'
        display_pose += '\t' + 'y: ' + str("%.4f" % result_pose.position.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" % result_pose.position.z) + '\n'
        display_pose += u'姿态:\n'
        display_pose += '\t' + 'X: ' + str("%.4f" % result_pose.orientation.x) + '\n'
        display_pose += '\t' + 'Y: ' + str("%.4f" % result_pose.orientation.y) + '\n'
        display_pose += '\t' + 'Z: ' + str("%.4f" % result_pose.orientation.z) + '\n'
        display_pose += '\t' + 'W: ' + str("%.4f" % result_pose.orientation.w) + '\n'

        self.text_pose.SetLabel(display_pose)

        # 显示关节角度
        result_joints_angles = self.limb.joint_angles()
        display_angles = ''
        for key, value in result_joints_angles.items():
            display_angles = key[6:8] + ':  ' + str("%.6f" % value) + '\n' + display_angles
        display_angles = u'关节角度：\n' + display_angles
        self.text_joint_angles.SetLabel(display_angles)

    def move_griper(self, event):
        #夹爪开合
        event_object = event.GetEventObject()
        action = event_object.GetLabel()
        if action == "close":
           self. gripper.close()
        elif action == "open":
            self.gripper.open()
        elif action == "calibrate":
            self.gripper.calibrate()



class mainApp(wx.App):
    def OnInit(self):
        self.SetAppName(WINDOWS_TITLE)
        self.Frame = mainFrame()
        self.Frame.Show()
        return True
class robot_init():
    def __init__(self):
        rp = intera_interface.RobotParams()
        valid_limbs = rp.get_limb_names()
        if not valid_limbs:
            rp.log_message(("Cannot detect any limb parameters on this robot. "
                            "Exiting."), "ERROR")
            return
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

if __name__ == "__main__":

    robot_init = robot_init()
    app = mainApp()
    app.MainLoop()

