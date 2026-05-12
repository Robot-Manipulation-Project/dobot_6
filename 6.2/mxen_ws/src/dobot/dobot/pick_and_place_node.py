import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from .dobot_client import DobotDriver
from .dobot_kinematics import inverse_kinematics
from dobot_interface.srv import PickAndPlace  # custom service interface
import time

CLEARANCE_Z = 100.0


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place_node")

        try:
            self.dobot = DobotDriver()
        except SystemExit:
            self.get_logger().error("Failed to connect to Dobot.")
            raise

        self.service = self.create_service(
            PickAndPlace,
            "pick_and_place",
            self.service_callback,
        )
        self.get_logger().info("Pick-and-place service server is ready.")

    def _pose_to_joints(self, x, y, z, r):
        q1, q2, q3, q4 = inverse_kinematics(x, y, z, r)
        return round(q1, 4), round(q2, 4), round(q3, 4), round(q4, 4)

    def _pose_is_reachable(self, x, y, z, r):
        try:
            q1, q2, q3, q4 = self._pose_to_joints(x, y, z, r)
        except Exception:
            return False
        return self.dobot.is_goal_valid(q1, q2, q3, q4)


    def service_callback(self, request, response):
        px, py, pz, pr = request.pick_pose
        dx, dy, dz, dr = request.place_pose

        if not self._pose_is_reachable(px, py, pz, pr):
            response.success = False
            response.message = (
                f"Pick pose ({px}, {py}, {pz}, {pr}) is outside the reachable workspace."
            )
            self.get_logger().warn(response.message)
            return response

        if not self._pose_is_reachable(dx, dy, dz, dr):
            response.success = False
            response.message = (
                f"Place pose ({dx}, {dy}, {dz}, {dr}) is outside the reachable workspace."
            )
            self.get_logger().warn(response.message)
            return response

        try:
            self.get_logger().info(
                f"Starting pick-and-place: "
                f"pick=({px},{py},{pz},{pr})  place=({dx},{dy},{dz},{dr})"
            )

            self.get_logger().info("Starting")
            q1, q2, q3, q4 = self._pose_to_joints(0, 207, 0, 203)

            self.get_logger().info("Move above pick position.")
            q1, q2, q3, q4 = self._pose_to_joints(px, py, CLEARANCE_Z, pr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)
            time.sleep(0.5)

            self.get_logger().info("Descend to pick position.")
            q1, q2, q3, q4 = self._pose_to_joints(px, py, pz, pr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)
            time.sleep(0.5)

            self.get_logger().info("Activate suction cup.")
            self.dobot.set_suction_cup(True)

            self.get_logger().info("Lift to clearance height.")
            q1, q2, q3, q4 = self._pose_to_joints(px, py, CLEARANCE_Z, pr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)

            self.get_logger().info("Move above place position.")
            q1, q2, q3, q4 = self._pose_to_joints(dx, dy, CLEARANCE_Z, dr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)

            self.get_logger().info("Descend to place position.")
            q1, q2, q3, q4 = self._pose_to_joints(dx, dy, dz, dr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)
            time.sleep(0.5)  

            self.get_logger().info("Deactivate suction cup.")
            self.dobot.set_suction_cup(False)

            self.get_logger().info("Retract to clearance height.")
            q1, q2, q3, q4 = self._pose_to_joints(dx, dy, CLEARANCE_Z, dr)
            self.dobot.set_joint_ptp(q1, q2, q3, q4)

            response.success = True
            response.message = "Pick-and-place completed successfully."
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"Error during pick-and-place: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    try:
        with rclpy.init(args=args):
            node = PickAndPlaceNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()