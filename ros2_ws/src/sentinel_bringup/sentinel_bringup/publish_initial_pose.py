# One-shot publisher: send a low-confidence initial pose (0, 0, 0) to
# AMCL, then exit. Launched by navigation.launch.py a few seconds after
# the nav2 stack starts so the user does not have to set the pose by
# hand. AMCL's own set_initial_pose param sends zero covariance (an
# overconfident point mass), which is why this node exists instead.
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

FRAME = 'map'
POSITION_VAR = 1.0   # ~1 m std: we do not know where the robot is
HEADING_VAR = 0.5    # ~0.7 rad std: heading likewise


def main():
    rclpy.init()
    node = rclpy.create_node('initial_pose_pub')
    pub = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    # Wait for AMCL (or any /initialpose subscriber) to come up.
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=60)
    while rclpy.ok() and pub.get_subscription_count() == 0:
        if node.get_clock().now() > deadline:
            node.get_logger().warn('No /initialpose subscriber after 60 s; giving up')
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = FRAME
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = POSITION_VAR
    msg.pose.covariance[7] = POSITION_VAR
    msg.pose.covariance[35] = HEADING_VAR
    pub.publish(msg)
    node.get_logger().info('Published initial pose (0, 0, 0), low confidence')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
