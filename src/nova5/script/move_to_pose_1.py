import rclpy
import time
import rclpy.duration
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.action import ActionClient
import rclpy.time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import sys, termios, tty
from tf_transformations import quaternion_from_euler
from tf_transformations import transforms3d
# import numpy as np  


def wait_for_space():
    print("\n按下空格执行笛卡尔轨迹...")
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

def wait_for_tf(buffer, target_frame, source_frame, timeout_sec=5):
    timeout = rclpy.duration.Duration(seconds=timeout_sec)
    start = rclpy.time.Time()
    while rclpy.ok():
        
        if buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout):
            return True
        now = rclpy.time.Time()
        if (now - start) > timeout:
            return False
        time.sleep(0.05)
    return False


def interpolate_waypoints(start_pose: Pose, end_pose: PoseStamped, steps=20):
    waypoints = []
    for i in range(steps + 1):
        pose = Pose()
        pose.position.x = start_pose.position.x + (end_pose.pose.position.x - start_pose.position.x) * i / steps
        pose.position.y = start_pose.position.y + (end_pose.pose.position.y - start_pose.position.y) * i / steps
        pose.position.z = start_pose.position.z + (end_pose.pose.position.z - start_pose.position.z) * i / steps
        # 这里姿态简单用目标姿态，你可以实现更复杂的插值（如slerp）
        pose.orientation = end_pose.pose.orientation
        waypoints.append(pose)
    return waypoints

class CartesianPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_planner')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

    def wait_for_services(self):
        self.get_logger().info("等待 /compute_cartesian_path 服务...")
        self.cartesian_client.wait_for_service()
        self.get_logger().info("等待 /execute_trajectory Action 服务...")
        self.execute_client.wait_for_server()

    def get_current_pose(self, target_frame='Link6', source_frame='base_link',timeout_sec=5.0):
        start_time = time.time()
        
        # self.get_logger().info("等待 TF 转换服务...")
        # if not wait_for_tf(self.tf_buffer, source_frame, target_frame,timeout_sec=5.0):
        #     self.get_logger().error("TF 转换服务未就绪！")
        #     return None
    
        
        while rclpy.ok():
            try:
                trans = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                pose = Pose()
                pose.position.x = trans.transform.translation.x
                pose.position.y = trans.transform.translation.y
                pose.position.z = trans.transform.translation.z
                pose.orientation = trans.transform.rotation
                return pose
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(f"无法获取机器人位置: {e}")
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error("超时")
                    return None
                time.sleep(0.001)
        # try:
        
        # except Exception as e:
        #     self.get_logger().error(f'获取当前末端位姿失败: {e}')
        #     return None

    def plan_cartesian_path(self, waypoints):
        req = GetCartesianPath.Request()
        req.group_name = 'nova5_arm'
        req.link_name = 'Link6'
        req.max_step = 0.01  # 插值步长，越小越平滑但越慢
        req.jump_threshold = 0.0  # 禁用跳跃检测
        req.avoid_collisions = True
        req.waypoints = waypoints
        req.header.frame_id = 'base_link'
        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("执行目标被拒绝 ❌")
            return False

        self.get_logger().info("开始执行轨迹...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("执行完成 ✅")
        return True

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPlanner()
    node.wait_for_services()

    # current_pose = node.get_current_pose()
    # if current_pose is None:
    #     node.get_logger().error("无法获取当前末端位姿，程序退出")
    #     rclpy.shutdown()
    #     return


    # 设置目标位姿
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
    # current.header.frame_id = 'base_link'
    # current.pose.position.x = current_pose[0]
    # current.pose.position.y = current_pose[1]
    # current.pose.position.z = current_pose[2]
    
    # current.pose.orientation.x = q_[0]
    # current.pose.orientation.y = q_[1]
    # current.pose.orientation.z = q_[2]
    # current.pose.orientation.w = q_[3]




    # 设置目标位姿
    target_pose = [-0.560, -0.230, 0.470, 1.546, 0.054, -1.663]  # [x,y,z,roll,pitch,yaw]
    
    target = Pose()
    # target.header.frame_id = 'base_link'
    target.position.x = target_pose[0]
    target.position.y = target_pose[1]
    target.position.z = target_pose[2]
    # r_mat = transforms3d.euler.euler2mat(target_pose[3], target_pose[4], target_pose[5])
    # q = transforms3d.quaternions.mat2quat(r_mat)
    # print('r_mat:',r_mat)
    q = quaternion_from_euler(target_pose[3], target_pose[4], target_pose[5])
    target.orientation.x = q[0]
    target.orientation.y = q[1]
    target.orientation.z = q[2]
    target.orientation.w = q[3]
    print('q: ',target.orientation)
    # waypoints = interpolate_waypoints(current, target, steps=20)
    # fraction = 0.0
    # while fraction <1.0:
    result = node.plan_cartesian_path([target])
    if not result:
        node.get_logger().error("笛卡尔路径规划失败，没有返回结果")
        rclpy.shutdown()
        return

    traj = result.solution
    fraction = result.fraction
    node.get_logger().info(f"规划路径覆盖率 fraction={fraction:.3f}")
    if fraction < 1.0:
        node.get_logger().warn("路径未完全规划成功，可能存在障碍或超出机械臂工作范围")

    input("按回车键执行轨迹...")
    node.execute_trajectory(traj)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
