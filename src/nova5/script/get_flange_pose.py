import rclpy
import tf2_ros
import math
import time

def main():
    rclpy.init()

    node = rclpy.create_node('GetLink6BaselinkPose')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer,node)

    time.sleep(1)

    try:
        trans = tf_buffer.lookup_transform(
            'base_link',
            'Link6',
            rclpy.time.Time()
        )

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z

        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w

        angle = 2 * math.acos(qw)
        s = math.sqrt(1 - qw*qw)

        if s < 1e-6:
            rx = qx
            ry = qy
            rz = qz
        else:
            rx = qx / s*angle
            ry = qy / s*angle
            rz = qz / s*angle

        pos = [x,y,z,rx,ry,rz]

    except Exception as e:
        node.get_logger().error(f'get TF failed: {e}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


