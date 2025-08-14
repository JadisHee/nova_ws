#!/usr/bin/env python3
import rclpy
import sys
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive


class MotionPlanner(Node):
    def __init__(self, group_name="nova5_arm", cartesian=False):
        super().__init__("motion_planner_client")

        self.group_name = group_name
        self.cartesian = cartesian

        self.move_group_client = ActionClient(self, MoveGroup, "move_action")
        self.cartesian_client = self.create_client(GetCartesianPath, "compute_cartesian_path")

        self.get_logger().info("等待 MoveGroup action server...")
        self.move_group_client.wait_for_server()

        if self.cartesian:
            self.get_logger().info("等待 CartesianPath 服务...")
            self.cartesian_client.wait_for_service()

    def create_pose_constraints(self, pose_stamped):
        constraints = Constraints()

        pos_const = PositionConstraint()
        pos_const.header = pose_stamped.header
        pos_const.link_name = "Link6"
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]
        pos_const.constraint_region.primitives.append(box)
        pos_const.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_const.weight = 1.0

        ori_const = OrientationConstraint()
        ori_const.header = pose_stamped.header
        ori_const.link_name = "Link6"
        ori_const.orientation = pose_stamped.pose.orientation
        ori_const.absolute_x_axis_tolerance = 0.01
        ori_const.absolute_y_axis_tolerance = 0.01
        ori_const.absolute_z_axis_tolerance = 0.01
        ori_const.weight = 1.0

        constraints.position_constraints.append(pos_const)
        constraints.orientation_constraints.append(ori_const)

        return constraints

    def trajectory_duration(self, joint_traj):
        if not joint_traj.points:
            return math.inf
        last_point = joint_traj.points[-1]
        return last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9

    def plan_motion(self, pose_target, attempts=5):
        best_trajectory = None
        best_points = math.inf
        best_duration = math.inf

        for i in range(attempts):
            self.get_logger().info(f"第 {i + 1}/{attempts} 次规划...")

            if self.cartesian:
                req = GetCartesianPath.Request()
                req.group_name = self.group_name
                req.waypoints.append(pose_target.pose)
                req.max_step = 0.01
                req.jump_threshold = 0.0
                req.avoid_collisions = True

                future = self.cartesian_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                res = future.result()

                if not res or not res.solution.joint_trajectory.points:
                    self.get_logger().warn("笛卡尔路径规划失败，跳过")
                    continue

                points_count = len(res.solution.joint_trajectory.points)
                duration = res.fraction  # fraction代表规划成功比例，非时间，用作辅助参考

                # 这里没时间信息，用路径点数做主参考
                score_points = points_count
                score_duration = 1.0 - duration  # fraction越大越好，转换成越小越好

                if score_points < best_points or (score_points == best_points and score_duration < best_duration):
                    best_trajectory = res.solution
                    best_points = score_points
                    best_duration = score_duration

            else:
                goal = MoveGroup.Goal()
                goal.request.group_name = self.group_name
                goal.request.goal_constraints.append(self.create_pose_constraints(pose_target))
                goal.request.allowed_planning_time = 5.0
                goal.planning_options.plan_only = True

                future = self.move_group_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, future)
                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn("规划请求被拒绝")
                    continue

                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result().result

                if not hasattr(result, "planned_trajectory") or not result.planned_trajectory.joint_trajectory.points:
                    self.get_logger().warn("规划结果为空，跳过")
                    continue

                points_count = len(result.planned_trajectory.joint_trajectory.points)
                duration = self.trajectory_duration(result.planned_trajectory.joint_trajectory)

                if points_count < best_points or (points_count == best_points and duration < best_duration):
                    best_trajectory = result.planned_trajectory
                    best_points = points_count
                    best_duration = duration

            self.get_logger().info(f"本次规划路径点数: {points_count}, 参考时间或分数: {duration}")

        return best_trajectory

    def execute_trajectory(self, trajectory):
        if not trajectory:
            self.get_logger().error("没有轨迹可执行！")
            return

        self.get_logger().info("按空格键后执行轨迹...")
        while True:
            ch = sys.stdin.read(1)
            if ch == " ":
                break

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name

        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = False
        # goal.planned_trajectory = trajectory




        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("执行请求被拒绝！")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("轨迹执行完成！")


def main(args=None):
    rclpy.init(args=args)
    planner = MotionPlanner(group_name="nova5_arm", cartesian=False)

    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = 0.4
    target.pose.position.y = 0.0
    target.pose.position.z = 0.6
    target.pose.orientation.w = 1.0

    best_traj = planner.plan_motion(target, attempts=5)

    if best_traj:
        planner.get_logger().info(
            f"选出最优轨迹：点数={len(best_traj.joint_trajectory.points)}, "
            f"预计时长={planner.trajectory_duration(best_traj.joint_trajectory):.3f}秒"
        )
        planner.execute_trajectory(best_traj)
    else:
        planner.get_logger().error("未找到可执行轨迹")

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
