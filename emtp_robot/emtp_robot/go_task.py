import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class WaypointNavigator(Node):

    def __init__(self, poses):
        super().__init__('waypoint_navigator')

        self.client = ActionClient(
            self,
            NavigateToPose,
            '/robot2/navigate_to_pose'
        )

        self.poses = poses
        self.goal_handle = None
        self.canceled = False

        # cancel 서비스
        self.cancel_srv = self.create_service(
            Trigger,
            'cancel_navigation',
            self.cancel_service_callback
        )

        self.send_next_goal()

    def send_next_goal(self):
        if self.canceled:
            self.get_logger().info('Navigation canceled. Exit.')
            rclpy.shutdown()
            return

        if not self.poses:
            self.get_logger().info('All goals finished')
            rclpy.shutdown()
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.poses.pop(0)
        self.client.wait_for_server()

        self.get_logger().info('Sending new goal')

        self.send_goal_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.send_next_goal()
            return

        self.get_logger().info('Goal accepted')

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):
        status = future.result().status

        if self.canceled:
            self.get_logger().info('Canceled, stop processing')
            rclpy.shutdown()
            return

        if status == 4:      # SUCCEEDED
            self.get_logger().info('Goal succeeded')
        elif status == 5:    # CANCELED
            self.get_logger().info('Goal canceled')
            rclpy.shutdown()
            return
        else:
            self.get_logger().warn(f'Goal ended with status {status}')

        self.send_next_goal()

    def cancel_service_callback(self, request, response):
        if self.goal_handle is None:
            response.success = False
            response.message = 'No active goal'
            return response

        self.get_logger().info('Cancel request received')
        self.canceled = True

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)

        response.success = True
        response.message = 'Cancel requested'
        return response

    def cancel_done(self, future):
        self.get_logger().info('Cancel sent to Nav2')

    def feedback_callback(self, feedback_msg):
        pass


def make_pose(x, y, z, w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def main():
    rclpy.init()

    poses = [
        make_pose(9.03401, 21.1768, 0.0, -0.00420319),
        make_pose(9.87, 20.7, 0.00247,0.0103),
        # make_pose(9.43, 20.5, 0.00247,-0.00420319),
    ]

    node = WaypointNavigator(poses)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
