import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_interface.action import PosePTP

from .dobot_client import DobotDriver
from .dobot_kinematics import forward_kinematics, inverse_kinematics


class CartesianPTPNode(Node):

    def __init__(self):
        super().__init__("cartesian_ptp_node")
        self.goal_handle = None
        self.goal_lock = threading.Lock()

        try:
            self.dobot = DobotDriver()
        except SystemExit:
            self.get_logger().error("Failed to connect to Dobot.")
            raise

        self.action_server = ActionServer(
            self,
            PosePTP,
            "set_cartesian_ptp",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    def _pose_to_joints(self, pose_goal):
        """Convert a Cartesian pose goal [x, y, z, r] to joint angles via IK.
        Rounds to 4 decimal places to avoid floating point edge cases (e.g. -0.0000)."""
        x, y, z, r = pose_goal
        q1, q2, q3, q4 = inverse_kinematics(x, y, z, r)
        return round(q1, 4), round(q2, 4), round(q3, 4), round(q4, 4)

    def _goal_is_reachable(self, pose_goal):
        """Return True if the pose is reachable and all joint limits are satisfied."""
        try:
            q1, q2, q3, q4 = self._pose_to_joints(pose_goal)
        except Exception:
            return False
        return self.dobot.is_goal_valid(q1, q2, q3, q4)

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        """Accept or reject an incoming goal based on workspace reachability."""
        self.get_logger().info("Received cartesian goal request")

        if self._goal_is_reachable(goal_request.pose_goal):
            return GoalResponse.ACCEPT

        self.get_logger().warn("Invalid cartesian goal: pose outside reachable workspace.")
        return GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle):
        """Abort any active goal, then execute the new one."""
        with self.goal_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                self.dobot.stop_current_action()
                self.goal_handle.abort()
            self.goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Stop the robot and accept the cancellation."""
        self.dobot.stop_current_action()
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Send the Cartesian goal to the robot and wait for completion."""
        self.get_logger().info("Executing cartesian goal...")

        goal_msg = goal_handle.request

        # Convert Cartesian goal to joint space and command the robot
        q1, q2, q3, q4 = self._pose_to_joints(goal_msg.pose_goal)
        self.dobot.set_joint_ptp(q1, q2, q3, q4)

        feedback_msg = PosePTP.Feedback()
        result = PosePTP.Result()

        # Thresholds in Cartesian space (mm for x/y/z, degrees for r)
        threshold_goal = [5.0, 5.0, 5.0, 10.0]

        # Use 10 Hz feedback rate (same as joint node)
        rate = self.create_rate(10)

        while rclpy.ok():
            if not goal_handle.is_active:
                self.get_logger().info("Goal aborted externally")
                result.success = False
                return result

            if goal_handle.is_cancel_requested:
                self.dobot.stop_current_action()
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result.success = False
                return result

            # Read current joint state and convert to Cartesian for feedback
            j1, j2, j3, j4 = self.dobot.get_joint_state()
            x, y, z, r = forward_kinematics(j1, j2, j3, j4)
            feedback_msg.pose_present = [x, y, z, r]
            goal_handle.publish_feedback(feedback_msg)

            # Check whether the end-effector has reached the goal pose
            goal_err = [
                abs(goal_msg.pose_goal[i] - feedback_msg.pose_present[i])
                for i in range(4)
            ]
            if all(goal_err[i] < threshold_goal[i] for i in range(4)):
                break

            rate.sleep()

        with self.goal_lock:
            if not goal_handle.is_active:
                return PosePTP.Result()
            goal_handle.succeed()

        result.success = True
        self.get_logger().info("Cartesian goal succeeded")
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = CartesianPTPNode()
            executor = MultiThreadedExecutor()
            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()