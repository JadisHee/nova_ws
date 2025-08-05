import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')

        # 创建 publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/position_controller/joint_trajectory',
            10
        )

        # 发送一次指令
        self.timer = self.create_timer(1.0, self.send_command)
        self.sent = False

    def send_command(self):
        if self.sent:
            return

        msg = JointTrajectory()
        msg.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.5, -1.3, 0.2, -1.0, 1.2, 0.0]  # ← 可更改为任意角度值（单位：弧度）
        point.time_from_start.sec = 2  # 2秒内完成动作

        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info('Joint trajectory command sent.')
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin_once(node, timeout_sec=3)  # 等待3秒发出并结束
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
