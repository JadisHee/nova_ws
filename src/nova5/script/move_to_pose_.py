import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler
import sys, termios, tty

def wait_for_space():
    print("\n按下空格执行规划路径...")
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == ' ':
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def get_current_pose():
    current_pose = [-0.560, -0.130, 0.470, 1.546, 0.054, -1.663]  # [x,y,z,roll,pitch,yaw]

    current = Pose()
    current.position.x = current_pose[0]
    current.position.y = current_pose[1]
    current.position.z = current_pose[2]
    q_ = quaternion_from_euler(current_pose[3], current_pose[4], current_pose[5])
    current.orientation.x = q_[0]
    current.orientation.y = q_[1]
    current.orientation.z = q_[2]
    current.orientation.w = q_[3]

    return current



class MoveIt2Planner(Node):
    def __init__(self, mode='normal'):
        super().__init__('moveit2_planner')

        self.mode = mode
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

    def wait_for_servers(self):
        self.get_logger().info('等待 action servers 和 services...')
        self._action_client.wait_for_server()
        self.execute_client.wait_for_server()
        self.cartesian_client.wait_for_service()
        self.get_logger().info('全部就绪')

    def plan_normal(self, pose_goal: PoseStamped, attempts=10):
        best_trajectory = None
        best_point_count = float('inf')
        best_time = float('inf')

        for i in range(attempts):
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = 'nova5_arm'
            goal_msg.planning_options.plan_only = True
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 2.0

            constraints = Constraints()
            pc = PositionConstraint()
            pc.header = pose_goal.header
            pc.link_name = 'Link6'
            pc.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001]*3))
            pc.constraint_region.primitive_poses.append(pose_goal.pose)
            pc.weight = 1.0

            oc = OrientationConstraint()
            oc.header = pose_goal.header
            oc.link_name = 'Link6'
            oc.orientation = pose_goal.pose.orientation
            oc.absolute_x_axis_tolerance = 0.001
            oc.absolute_y_axis_tolerance = 0.001
            oc.absolute_z_axis_tolerance = 0.001
            oc.weight = 1.0

            constraints.position_constraints.append(pc)
            constraints.orientation_constraints.append(oc)
            goal_msg.request.goal_constraints.append(constraints)

            self.get_logger().info(f'普通规划第 {i+1}/{attempts} 次...')
            future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'普通规划第 {i+1} 次被拒绝')
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result

            if result.error_code.val != 1:
                self.get_logger().warn(f'普通规划第 {i+1} 次失败')
                continue

            traj = result.planned_trajectory
            point_count = len(traj.joint_trajectory.points)
            total_time = traj.joint_trajectory.points[-1].time_from_start.sec + \
                         traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9

            self.get_logger().info(f'普通规划第 {i+1} 次: 路径点={point_count}, 用时={total_time:.3f}s')

            if point_count < best_point_count or (point_count == best_point_count and total_time < best_time):
                best_trajectory = traj
                best_point_count = point_count
                best_time = total_time

        if best_trajectory:
            self.get_logger().info(f'普通规划最优路径: 路径点={best_point_count}, 用时={best_time:.3f}s')
        else:
            self.get_logger().error('普通规划所有尝试失败')
        return best_trajectory

    def plan_cartesian(self, waypoints, attempts=10):
        best_trajectory = None
        best_point_count = float('inf')
        best_time = float('inf')

        req = GetCartesianPath.Request()
        req.group_name = 'nova5_arm'
        req.link_name = 'Link6'
        req.max_step = 0.1
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.header.frame_id = 'base_link'

        for i in range(attempts):
            req.waypoints = waypoints

            self.get_logger().info(f'笛卡尔规划第 {i+1}/{attempts} 次...')
            future = self.cartesian_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            if not resp:
                self.get_logger().warn(f'笛卡尔规划第 {i+1} 次无响应')
                continue

            traj = resp.solution
            fraction = resp.fraction
            if fraction < 1.0:
                self.get_logger().warn(f'笛卡尔规划第 {i+1} 次部分成功 fraction={fraction:.2f}')
            else:
                self.get_logger().info(f'笛卡尔规划第 {i+1} 次成功 fraction=1.0')

            point_count = len(traj.joint_trajectory.points)
            total_time = traj.joint_trajectory.points[-1].time_from_start.sec + \
                         traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9

            self.get_logger().info(f'笛卡尔规划第 {i+1} 次: 路径点={point_count}, 用时={total_time:.3f}s')

            if fraction == 1.0 and (point_count < best_point_count or (point_count == best_point_count and total_time < best_time)):
                best_trajectory = traj
                best_point_count = point_count
                best_time = total_time

        if best_trajectory:
            self.get_logger().info(f'笛卡尔规划最优路径: 路径点={best_point_count}, 用时={best_time:.3f}s')
        else:
            self.get_logger().error('笛卡尔规划所有尝试失败或未完全成功')
        return best_trajectory

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("执行目标被拒绝 ❌")
            return

        self.get_logger().info("开始执行轨迹...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("执行完成 ✅")



def main(target_pose,mod):
    import sys
    rclpy.init(args=None)

    # 从命令行获取规划模式 normal/cartesian，默认normal
    if mod == 0:
        mode = 'normal'
    elif mod == 1:
        mode = 'cartesian'
    else:
        mode = 'normal'


    if len(sys.argv) > 1:
        mode = sys.argv[1]
    print(f"规划模式: {mode}")

    node = MoveIt2Planner(mode=mode)
    node.wait_for_servers()
    
    import time
    start_t = time.time()
    
    # 设置目标位姿

    # 位置_1
    # target_pose = [-0.486758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]

    # 位置_2
    # target_pose = [-0.486758423,-0.130443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]

    # 目标位置
    # target_pose = [-0.586758423,-0.130443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]

    # 位置_3
    # target_pose = [-0.586758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]

    # 位置_4
    # target_pose = [-0.486758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.pose.position.x = target_pose[0]
    target.pose.position.y = target_pose[1]
    target.pose.position.z = target_pose[2]
    q = quaternion_from_euler(target_pose[3], target_pose[4], target_pose[5])
    target.pose.orientation.x = q[0]
    target.pose.orientation.y = q[1]
    target.pose.orientation.z = q[2]
    target.pose.orientation.w = q[3]

    if mode == 'normal':
        traj = node.plan_normal(target, attempts=10)
    else:
        # 笛卡尔模式，构造waypoints示例，简单从当前位姿插值到目标
        waypoints = [target.pose]  # 你也可以传入多段路径
        traj = node.plan_cartesian(waypoints, attempts=10)

    end_t = time.time()
    dt = end_t - start_t
    print('总共规划耗时:',dt)
    if traj:
        wait_for_space()
        node.execute_trajectory(traj)

    node.destroy_node()
    rclpy.shutdown()


def wait_for_target(args=Node):
    rclpy.init(args=args)
    node = rclpy.create_node('TargetPose_subscriber')  
    



if __name__ == '__main__':
    import math
    # 位置_1
    target_pose_1 = [-0.486758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_1,1)
    # 位置_2
    target_pose_2 = [-0.486758423,-0.130443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_2,1)
    # 目标位置
    target_pose = [-0.586758423,-0.130443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose,1)
    # 位置_3
    target_pose_3 = [-0.586758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_3,1)
    # 位置_4
    target_pose_4 = [-0.586758423,-0.030443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_4,1)
    # 位置_5
    target_pose_5 = [-0.586758423,-0.030443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_5,1)

    # 位置_6
    target_pose_6 = [-0.486758423,-0.030443741,0.802750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_6,1)

    # 位置_1
    target_pose_1 = [-0.486758423,-0.130443741,0.702750366, -31.712141 * math.pi / 180,88.231888* math.pi / 180,151.753174* math.pi / 180]
    main(target_pose_1,1)
