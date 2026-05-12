# The following imports are necessary
import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dobot_interface.action import JointPTP
from .dobot_client import DobotDriver

class JointPTPNode(Node):

    def __init__(self):
        super().__init__("joint_ptp_node")
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        
        try:
            self.dobot = DobotDriver()
        except SystemExit:
            self.get_logger().error("Failed to connect to Dobot.")
            raise

        # Action servers are created using interface type, action name and multiple callback functions
        self.action_server = ActionServer(
            self,
            JointPTP,
            "set_joint_ptp",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self.action_server.destroy()
        super().destroy_node()

    # This function is called whenever new goal request is received
    def goal_callback(self, goal_request):
        # Accept or reject a client request to begin an action.
        self.get_logger().info("Received goal request")
        
        if self.dobot.is_goal_valid(
            goal_request.joint_goal[0],
            goal_request.joint_goal[1],
            goal_request.joint_goal[2],
            goal_request.joint_goal[3]
        ):
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn("Invalid joint goal requested.")
            return GoalResponse.REJECT

    # This function is called whenever new goal has been accepted
    def handle_accepted_callback(self, goal_handle):
        with self.goal_lock:
            # This server only allows one goal at a time
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("Aborting previous goal")
                self.dobot.stop_current_action()
                # Abort the existing goal
                self.goal_handle.abort()
            self.goal_handle = goal_handle

        goal_handle.execute()

    # This function is called whenever cancel request is received
    def cancel_callback(self, goal):
        self.dobot.stop_current_action()
        # Accept or reject a client request to cancel an action.
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # This function is called at the start of action execution
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        
        goal_msg = goal_handle.request
        self.dobot.set_joint_ptp(
            goal_msg.joint_goal[0], 
            goal_msg.joint_goal[1], 
            goal_msg.joint_goal[2], 
            goal_msg.joint_goal[3]
        )

        feedback_msg = JointPTP.Feedback()
        result = JointPTP.Result()
        
        threshold_goal = 2.0 # Degrees
        threshold_progress = 0.01 # Degrees

        prev_present = [None, None, None, None]

        # Use 10Hz feedback rate
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

            # Get present joints and publish feedback
            j1, j2, j3, j4 = self.dobot.get_joint_state()
            feedback_msg.joint_present = [j1, j2, j3, j4]
            goal_handle.publish_feedback(feedback_msg)

            # Calculate differences from goal
            goal_err = [abs(goal_msg.joint_goal[i] - feedback_msg.joint_present[i]) for i in range(4)]
            if all(err < threshold_goal for err in goal_err):
                # goal reached
                break

            # Check if stuck
            if prev_present[0] is not None:
                step_diffs = [abs(feedback_msg.joint_present[i] - prev_present[i]) for i in range(4)]
                if all(diff < threshold_progress for diff in step_diffs):
                    self.get_logger().info("Robot stopped moving. Finishing action.")
                    break
            
            prev_present = [j1, j2, j3, j4]
            
            rate.sleep()

        with self.goal_lock:
            if not goal_handle.is_active:
                return JointPTP.Result()
            goal_handle.succeed()
            
        result.success = True
        self.get_logger().info("Goal succeeded")
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = JointPTPNode()

            # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
            executor = MultiThreadedExecutor()
            rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()