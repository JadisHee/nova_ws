from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    package_path = get_package_share_directory('nova5_moveit')

    moveit_config = MoveItConfigsBuilder("nova5_robot", package_name="nova5_moveit").to_moveit_configs()

    robot_desc_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name="static_transform_publisher",
        output='log',
        arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', package_path+'/config/moveit.rviz'],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits
        ]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[package_path+'/config/ros2_controllers.yaml'],
        output='both'
    )
    
    controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster','arm_controller','hand_controller'
        ]
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()]
    )

    return LaunchDescription([
        robot_desc_node,
        static_tf_node,
        rviz_node,
        ros2_control_node,
        controller_spawner_node,
        move_group_node
    ])