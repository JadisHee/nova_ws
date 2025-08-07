from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nova5_moveit_path = get_package_share_directory('nova5_moveit')

    # ✅ 启动 MoveIt + RViz（不含 ros2_control）
    moveit_without_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nova5_moveit_path, 'launch', 'move_group.launch.py')
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nova5_moveit_path, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # ✅ 启动 robot_state_publisher（加载 URDF）
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nova5_moveit_path, 'launch', 'rsp.launch.py')
        )
    )

    # ✅ 启动 joint_states 转发节点（真实机械臂 → /joint_states）
    joint_states_relay = Node(
        package='dobot_moveit',
        executable='joint_states',
        name='joint_states_relay'
    )

    # ✅ 启动 FollowJointTrajectory Action Server（MoveIt → 真实机械臂）
    action_server = Node(
        package='dobot_moveit',
        executable='action_move_server',
        name='dobot_group_controller'
    )

    return LaunchDescription([
        rsp,
        moveit_without_control,
        rviz,
        joint_states_relay,
        action_server,
    ])
