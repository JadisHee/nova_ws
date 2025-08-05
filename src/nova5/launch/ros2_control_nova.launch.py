import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 获取默认的urdf路径
    urdf_package_path = get_package_share_directory('nova5')
    default_urdf_path = os.path.join(urdf_package_path,'urdf/system','system.xacro')
    default_rviz_path = os.path.join(urdf_package_path,'rviz','display_nova5.rviz')
    controllers_yaml_path = os.path.join(urdf_package_path, 'config', 'nova5_ros2_controllers.yaml')

    # 声明一个urdf目录的参数，以便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_urdf_path),
        description='launch the urdf path'
    )

    # 获取 xacro 生成的 robot_description
    substitutions_command_result = launch.substitutions.Command([
        'xacro ',
        launch.substitutions.LaunchConfiguration('model')
    ])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result,
        value_type=str
    )

    # robot_state_publisher
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    # ros2_control_node
    action_ros2_control = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_value},
            controllers_yaml_path
        ],
        output='both'
    )

    # 分别加载 joint_state_broadcaster 和 position_controller
    load_joint_state_broadcaster = launch_ros.actions.Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster', '--controller-manager-timeout', '50'],
    output='screen'
)

    load_position_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager-timeout', '50'],
        output='screen'
    )


    # rviz
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_ros2_control,
        load_joint_state_broadcaster,
        load_position_controller,
        action_rviz_node
    ])
