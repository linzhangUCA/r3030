import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class SequentialNav2Goals(Node):
    def __init__(self):
        super().__init__("sequential_nav2_goals")
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goals = [
            {
                "x": 3.001382827758789,
                "y": 0.9243518710136414,
                "qz": -0.9482917594105156,
                "qw": 0.31739996697244444,
            },  # Goal 1
            {
                "x": 2.835209846496582,
                "y": 3.0596871376037598,
                "qz": 0.877063483057177,
                "qw": 0.4803744858832669,
            },  # Goal 2
            {
                "x": 0.4911618232727051,
                "y": 2.8559165000915527,
                "qz": -0.9433666010875643,
                "qw": 0.3317521001478308,
            },  # Goal 3
            {
                "x": 0.2277531623840332,
                "y": 0.2903428077697754,
                "qz": -0.924764526420679,
                "qw": 0.380539841112514,
            },  # Goal 4
        ]
        self.current_goal_index = 0

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("All goals completed!")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goals[self.current_goal_index]["x"]
        goal_msg.pose.pose.position.y = self.goals[self.current_goal_index]["y"]
        goal_msg.pose.pose.orientation.z = self.goals[self.current_goal_index]["qz"]
        goal_msg.pose.pose.orientation.w = self.goals[self.current_goal_index]["qw"]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal {self.current_goal_index} rejected!")
            return
        self.get_logger().info(f"Goal {self.current_goal_index} accepted!")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} meters"
        )

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f"Goal {self.current_goal_index} reached!")
            self.current_goal_index += 1
            self.send_next_goal()  # Proceed to next goal
        else:
            self.get_logger().info(
                f"Goal {self.current_goal_index} failed! Retrying..."
            )
            self.send_next_goal()  # Retry current goal


def main(args=None):
    rclpy.init(args=args)
    nav2_goals = SequentialNav2Goals()
    nav2_goals.send_next_goal()  # Start the sequence
    rclpy.spin(nav2_goals)


if __name__ == "__main__":
    main()
