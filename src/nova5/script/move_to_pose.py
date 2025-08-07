import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration

import sys
import select
import termios
import tty

class MoveToPoseClient(Node):
    def __init__(self):
        super().__init__('move_to_pose_client')

        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        self.planning_frame = 'base_link'
        self.group_name = 'nova5_arm'
        self.ee_link = 'Link6'  # 替换为你自己的末端 link 名称

    def send_goal(self, x, y, z, rx, ry, rz):
        self.get_logger().info('等待 MoveGroup action 服务器...')
        self._action_client.wait_for_server()

        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = quaternion_from_euler(rx, ry, rz)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.goal_constraints.append(self.create_goal_constraint(pose))

        # 第一次：仅规划，不执行
        goal_msg = MoveGroup.Goal()
        goal_msg.request = request
        goal_msg.planning_options.plan_only = True
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3

        self.get_logger().info('发送规划请求...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.plan_only_done_callback)

    def plan_only_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('路径规划请求被拒绝')
            return

        self.get_logger().info('路径规划成功，等待结果...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.execute_after_user_input)

    def execute_after_user_input(self, future):
        result = future.result().result

        if result.error_code.val != result.error_code.SUCCESS:
            self.get_logger().error(f'规划失败，错误码: {result.error_code.val}')
            return

        self.get_logger().info('规划成功 ✅')
        self.get_logger().info('按空格键执行运动，或按 Ctrl+C 退出')

        # 等待用户按空格键
        wait_for_space()

        # 执行之前的规划结果
        self.get_logger().info('正在执行轨迹...')

        # 创建执行请求（不再重新规划）
        exec_goal_msg = MoveGroup.Goal()
        exec_goal_msg.request = result.trajectory_start  # 仍需包含初始状态
        exec_goal_msg.planning_options.plan_only = False
        exec_goal_msg.planning_options.replan = False

        # 关键：设置 trajectory
        exec_goal_msg.trajectory = result.planned_trajectory
        exec_goal_msg.request.group_name = self.group_name

        send_exec_future = self._action_client.send_goal_async(exec_goal_msg)
        send_exec_future.add_done_callback(self.exec_done_callback)

    def exec_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('执行请求被拒绝')
            return

        self.get_logger().info('执行请求已接受，等待结果...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.exec_result_callback)

    def exec_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info('✅ 轨迹执行成功')
        else:
            self.get_logger().error(f'❌ 执行失败，错误码: {result.error_code.val}')

    def create_goal_constraint(self, pose: PoseStamped):
        constraints = Constraints()
        constraints.name = "goal_pose"

        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = self.ee_link

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose.header
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        return constraints

def wait_for_space():
    print('\n--- 请按下【空格键】开始执行 ---')
    old_attrs = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
                if key == ' ':
                    print('✅ 收到空格指令，开始执行')
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPoseClient()

    # 示例目标位姿
    target_pose = [-0.5615372, -0.1361008, 0.7732302, 0.0, 0.0, -1.57]
    node.send_goal(*target_pose)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()