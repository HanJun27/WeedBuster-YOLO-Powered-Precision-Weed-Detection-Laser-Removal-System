#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
strike_bringup.launch.py —— 一键起全链路  v3.11.2 新增
=====================================================
把 README 里"开 6 个终端按顺序手起"的流程固化成一条命令,拍视频/现场演示
反复重启时省时间、防起错顺序。

用法(先在本终端 source 一次即可):
    source ~/yahboomcar_ws/install/setup.bash
    ros2 launch laser_calibration strike_bringup.launch.py            # 全链路(含车控)
    ros2 launch laser_calibration strike_bringup.launch.py chassis:=false   # 台架调试,不起车控

⚠️ 亚博底盘驱动(Mcnamu_driver)属于厂商包、不在本包内,仍需单独先起
   (即键盘遥控能开动小车的那套 bringup)。只做台架打击演示(chassis:=false)
   时不需要它。

节点按依赖顺序错峰启动(模拟手动流程,避免 vision_servo 启动自检时相机
话题还没起来之类的竞态):
    0s  stereo_camera        相机驱动 + ISP 锁定
    3s  yolo_detector        BPU 推理(模型加载需几秒)
    6s  vision_servo         执行层(启动自检需要相机帧)
    9s  strike_planner       决策层
   12s  chassis_controller   车控(可选;上电默认 STANDBY,发 /chassis/start 才动 —— 安全默认)

全部起来后:
    浏览器开 http://<车IP>:8093 看伺服页面(含 v3.11.2 战报/决策队列面板)
    台架清场:  ros2 topic pub --once /planner/start_clearing std_msgs/msg/Empty {}
    整车发车:  ros2 topic pub --once /chassis/start std_msgs/msg/Empty {}
    全局急停:  ros2 topic pub --once /safety_stop std_msgs/msg/Empty {}
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    chassis = LaunchConfiguration("chassis")

    return LaunchDescription([
        DeclareLaunchArgument(
            "chassis", default_value="true",
            description="是否启动车控节点(台架调试可设 false)"),

        # 0s: 相机驱动
        Node(package="laser_calibration", executable="stereo_camera",
             name="stereo_camera", output="screen"),

        # 3s: YOLO 检测(BPU 模型加载需要几秒)
        TimerAction(period=3.0, actions=[
            Node(package="laser_calibration", executable="yolo_detector",
                 name="yolo_detector", output="screen"),
        ]),

        # 6s: 执行层(启动自检依赖相机帧)
        TimerAction(period=6.0, actions=[
            Node(package="laser_calibration", executable="vision_servo",
                 name="vision_servo", output="screen"),
        ]),

        # 9s: 决策层
        TimerAction(period=9.0, actions=[
            Node(package="laser_calibration", executable="strike_planner",
                 name="strike_planner", output="screen"),
        ]),

        # 12s: 车控(可选;STANDBY 静止,须手动 /chassis/start 才动)
        TimerAction(period=12.0, actions=[
            Node(package="laser_calibration", executable="chassis_controller",
                 name="chassis_controller", output="screen",
                 condition=IfCondition(chassis)),
        ]),
    ])
