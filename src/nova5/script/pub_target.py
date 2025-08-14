import rclpy
from geometry_msgs.msg import Pose

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('target_pos_publisher')


    publisher = node.create_publisher(Pose, '/target/tool_pose', 10)

    # 等待订阅者
    while rclpy.ok():
        sub_count = publisher.get_subscription_count()
        if sub_count > 0:
            msg = Pose()
            # msg.position.x = -0.091
            msg.position.x = 0.091
            msg.position.y = -0.568
            msg.position.z = 0.670
            msg.orientation.x = 0.698
            msg.orientation.y = 0.012
            msg.orientation.z = -0.026
            msg.orientation.w = 0.716
            publisher.publish(msg)
            node.get_logger().info(f'已发布位姿: {msg}')
            break
        else:
            node.get_logger().info('等待订阅者...')
            rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
