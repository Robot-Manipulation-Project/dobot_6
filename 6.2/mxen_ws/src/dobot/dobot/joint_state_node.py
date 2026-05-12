import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from .dobot_client import DobotDriver
from .dobot_kinematics import forward_kinematics


class JointStateAction(Node):

    def __init__(self):
        super().__init__("joint_state_node")
        self.dobot = None
        self.publisher = self.create_publisher(JointState, "joint_state", 10)
        self.pose_publisher = self.create_publisher(Pose, "pose", 10)
        timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def yaw_to_quaterion(self, yaw_degrees):
        yaw_radians = math.radians(yaw_degrees)
        half_yaw = yaw_radians / 2.0
        axis_x, axis_y, axis_z = 0.0, 0.0, 1.0
        sin_half = math.sin(half_yaw)
        return (
            axis_x * sin_half,
            axis_y * sin_half,
            axis_z * sin_half,
            math.cos(half_yaw),
        )

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = ["joint1", "joint2", "joint3", "joint4"]
        pose_msg = Pose()

        try:
            self.dobot = DobotDriver()
            j1, j2, j3, j4 = self.dobot.get_joint_state()
            msg.position = [j1, j2, j3, j4]
            self.publisher.publish(msg)
            x, y, z, r = forward_kinematics(j1, j2, j3, j4)
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            (
                pose_msg.orientation.x,
                pose_msg.orientation.y,
                pose_msg.orientation.z,
                pose_msg.orientation.w,
            ) = self.yaw_to_quaterion(r)
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f"Publishing: {msg.position}")

        except Exception as e:
            self.dobot = None
            self.get_logger().error(f"Can't connect to dobot: {e}")
            msg.position = [0.0, 0.0, 0.0, 0.0]
            self.publisher.publish(msg)
            pose_msg.position.x = 0.0
            pose_msg.position.y = 0.0
            pose_msg.position.z = 0.0
            pose_msg.orientation.w = 1.0
            self.pose_publisher.publish(pose_msg)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = JointStateAction()
            rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()