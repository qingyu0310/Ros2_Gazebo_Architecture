"""起仿真：gz 世界 + spawn 轮腿机器人 + 过桥。

    ros2 launch project sim.launch.py
    ros2 launch project sim.launch.py gz_args:='-s -r <world>'   # 只跑服务端，不开界面
        （gz_args 一传就把默认的 "-r <world>" 整个覆盖掉，world 得自己带上）

    拉起来的东西：gz、robot_state_publisher、spawn、桥接，加上 chassis（平衡环 + 两个电机）。
    imu 那条节点还停在下面（注释掉了）—— 平衡环直接订 /imu 自己取俯仰，中间不需要它。

    这条链上各环管什么（节点解冻后照旧）：
      gz 侧   gazebo/worlds/empty.sdf：Physics（物理）+ SceneBroadcaster（位姿）+ 传感器那几路
      模型    wheel_leg_robot：底盘 + 两条腿（胯/大腿/膝/小腿）+ 两个轮子。轮关节上挂
              ApplyJointForce（力矩入口），joint_encoder 宏在模型上挂 JointStatePublisher
              （关节角速度反馈）—— 这个插件只能挂模型，挂世界上 gz 会报
              "attached to a model entity" 且静默不干活
      桥      两个力矩入口 ROS->gz、关节状态 gz->ROS、IMU gz->ROS、时钟 gz->ROS
      节点    chassis（一个进程三个节点：平衡环 + 两个电机，参数走 params/chassis.yaml）
              imu 停用中：它那套姿态解算没人用
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# 两个轮子，名字跟 wheel_leg_robot.xacro 里的一致：模型名 = 车名，关节名 = <轮子>_wheel_joint
MODEL_NAME = 'wheel_leg_robot'
WHEELS = ['left', 'right']

# 轮子半径，见 project 的 modules/wheel.xacro。spawn 的 z 填它，两个轮子正好同时贴地：
# 底盘原点在轮轴高度上，腿在标准姿态下（α = 40°）把轮轴顶到胯轴下方 72mm 处
SPAWN_Z = '0.04'


def generate_launch_description():
    robot_xacro = os.path.join(get_package_share_directory('project'), 'robot', 'models', 'bodys',
                               'wheel_leg_robot.xacro')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    # 底盘参数：chassis 自己那份（模型尺寸、LQR 权重、限幅、遥控接口），外加两个电机走的
    # /** 和 /motor_left、/motor_right 段 —— 电机是同一个进程里的子节点，会把整份文件一起读
    chassis_params = os.path.join(get_package_share_directory('project'), 'params', 'chassis.yaml')

    # 用 gazebo 层自带的世界：里面挂了 Physics/SceneBroadcaster/JointStatePublisher 这些系统。
    # gz 装的那份 empty.sdf 只有前两个，关节状态和传感器都没东西发。
    world = PathJoinSubstitution([FindPackageShare('gazebo'), 'worlds', 'empty.sdf'])

    # xacro 展开成 URDF，由 robot_state_publisher 发到 /robot_description
    robot_description = ParameterValue(Command(['xacro ', robot_xacro]), value_type=str)

    # 力矩入口：ROS 侧是 /motor/<轮子>/cmd_force，
    # gz 侧是 /model/<模型名>/joint/<关节名>/cmd_force（ApplyJointForce 自己拼的，名字里焊死了模型名）。
    # 两边名字对不上，所以每条桥配一个重映射。
    # 方向 ] = ROS->gz，[ = gz->ROS（见 parameter_bridge --help）
    bridge_args = []
    remaps = []
    for wheel in WHEELS:
        gz_topic = f'/model/{MODEL_NAME}/joint/{wheel}_wheel_joint/cmd_force'
        bridge_args.append(f'{gz_topic}@std_msgs/msg/Float64]gz.msgs.Double')
        remaps.append(f'{gz_topic}:=/motor/{wheel}/cmd_force')

    # 关节状态：话题名是模型里 joint_encoder 宏写死的 /joint_states（不是 gz 按 世界/模型 拼的），
    # 跟 ROS 侧同名，桥上不用动
    bridge_args.append('/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model')

    # IMU：传感器在模型里（wheel_leg_robot 的 base_link，见 gazebo/plugins/sensor/imu.xacro），
    # gz 侧就是 /imu（绝对话题名，不会加 世界/模型 前缀）。两边同名，不用重映射
    bridge_args.append('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')

    # 仿真时钟：chassis 和两个电机都用 use_sim_time，
    # 没有 /clock 的话 now() 一直是 0，速度环的 dt 也就是 0，积分项永远不攒
    bridge_args.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

    remap_args = []
    for remap in remaps:
        remap_args += ['-r', remap]

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=['-r ', world],
                              description='传给 gz sim 的参数；加 -s 只跑服务端，不开界面'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
             output='screen'),

        # z 给轮子半径：底盘原点在轮轴高度上，这样两个轮子正好落在 z=0 的地面上
        Node(package='ros_gz_sim',
             executable='create',
             arguments=['-topic', 'robot_description', '-name', MODEL_NAME, '-z', SPAWN_Z],
             output='screen'),

        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             arguments=bridge_args + ['--ros-args'] + remap_args,
             output='screen'),

        # ---- 节点 ----
        #
        # 底盘：一个进程里三个节点（chassis + left/right 两个电机）。chassis 订 /imu、
        # /joint_states、/chassis/cmd_vel，算完把力矩发给自己那两个电机，电机再发到桥那边。
        # 参数全走 params/chassis.yaml；改了那个文件要 colcon build 才会同步到 install
        Node(package='project',
             executable='chassis',
             parameters=[chassis_params, {'use_sim_time': True}],
             output='screen'),

        # IMU 解算：订阅桥过来的 /imu，发 /imu/attitude、/imu/quaternion、/imu/euler_* 那些。
        # 平衡环不需要它：chassis 直接订 /imu 取俯仰和角速度，这里再解一遍是白算
        # Node(package='project',
        #      executable='imu',
        #      parameters=[{'use_sim_time': True}],
        #      output='screen'),
    ])
