import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__("nav2_goal_client")
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = 3.00
        goal_msg.pose.pose.position.y = 0.92
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected!")
            return
        self.get_logger().info("Goal accepted!")


def main(args=None):
    rclpy.init(args=args)
    goal_node = Nav2GoalClient()
    goal_node.send_goal()
    rclpy.spin(goal_node)  # Will exit immediately due to shutdown in publish_once
    rclpy.shutdown()


if __name__ == "__main__":
    main()
