import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from .dobot_client import DobotDriver


class HomingService(Node):

    def __init__(self):
        super().__init__("homing_node")
        self.dobot = None
        self.service = self.create_service(Trigger, "homing", self.service_callback)
        self.get_logger().info("Homing service server is ready.")


    def service_callback(self, request, response):
        try:
            self.dobot = DobotDriver()
            self.get_logger().info("Starting homing procedure...")
            self.dobot.start_homing()
            response.success = True
            response.message = "Homing completed successfully"
            self.get_logger().info("Homing completed.")
        except Exception as e:
            self.dobot = None
            response.success = False
            response.message = f"Homing failed: {str(e)}"
            self.get_logger().error(f"Homing error: {str(e)}")
        return response


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = HomingService()

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()