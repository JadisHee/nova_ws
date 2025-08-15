import rclpy
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler

def pub_target(target_pose):
    rclpy.init(args=None)
    node = rclpy.create_node('target_pos_publisher')


    publisher = node.create_publisher(Pose, '/target/tool_pose', 10)

    # 等待订阅者
    while rclpy.ok():
        sub_count = publisher.get_subscription_count()
        if sub_count > 0:
            msg = Pose()
            # msg.position.x = -0.091
            msg.position.x = target_pose[0]
            msg.position.y = target_pose[1]
            msg.position.z = target_pose[2]
            q = quaternion_from_euler(target_pose[3],target_pose[4],target_pose[5])
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            publisher.publish(msg)
            node.get_logger().info(f'已发布位姿: {msg}')
            break
        else:
            node.get_logger().info('等待订阅者...')
            rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    flange_target_photo_pose = [-0.651, -0.104, 0.416, 0.027, 0.002, -0.002]
    pub_target(flange_target_photo_pose)



    flange_target_get_it_ready_in = [-0.723, -0.111, 0.484, 0.084, 0.061, -0.803]
    flange_target_get_it = [-0.725, -0.124, 0.614, 0.084, 0.061, -0.803]
    # flange_target_get_it_ready_out = []



    flange_target_waypoint_1 = [-0.596, -0.416, 0.541, 0.005, 0.032, 0.001]
    flange_target_waypoint_2 = [0.470, -0.415, 0.506, 0.005, 0.032, 0.001]
    flange_target_put_it_ready = [0.453, 0.048, 0.509, 0.032, -0.006, 1.608]
    flange_target_put_it = [-0.045, -0.211, 1.090, -1.571, -0.001, 2.614]
    # main(flange_target_put_it_down_ready)
