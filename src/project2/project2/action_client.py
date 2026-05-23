import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_interfaces.action import Interface
import yaml


class NavigationActionClient(Node):

    def __init__(self):
        super().__init__('navigation_action_client')

        self.declare_parameter('route_points', '[]')
        route_param_value = self.get_parameter('route_points').value

        try:
            self.route_data = yaml.safe_load(route_param_value)
            self.get_logger().info(f'Route data loaded: {self.route_data}')
        except Exception as parse_error:
            self.get_logger().error(f'Failed to parse route data: {parse_error}')
            self.route_data = []

        self.flattened_route = self.flatten_route_data(self.route_data)
        self.action_client = ActionClient(self, Interface, 'navigate_action')

        self.send_navigation_goal(self.flattened_route)

    def flatten_route_data(self, route):
        flat_route = []
        for waypoint in route:
            normalized_waypoint = {
                key.lower(): waypoint.get(key, 0.0) for key in ['x', 'y', 'z']
            }
            flat_route.extend([
                float(normalized_waypoint['x']), 
                float(normalized_waypoint['y']), 
                float(normalized_waypoint['z'])
            ])
        return flat_route

    def send_navigation_goal(self, route):
        goal_message = Interface.Goal()
        goal_message.route = route

        self.action_client.wait_for_server()

        self.future_goal = self.action_client.send_goal_async(
            goal_message, 
            feedback_callback=self.handle_feedback_update
        )
        self.future_goal.add_done_callback(self.process_goal_response)

    def process_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('The goal was not accepted by the action server.')
            return

        self.get_logger().info('The goal has been accepted.')
        self.future_result = goal_handle.get_result_async()
        self.future_result.add_done_callback(self.process_result)

    def process_result(self, future):
        result_code = int(future.result().result.result)
        result_messages = {
            0: 'Goal execution succeeded!',
            1: 'The route is empty.',
            2: 'No initial pose detected.',
            3: 'Goal was canceled.',
            4: 'Goal execution failed.',
            5: 'Invalid return status from the server.'
        }

        result_message = result_messages.get(result_code, 'Received an unknown result code.')
        self.get_logger().info(f'Result: {result_message}')

        rclpy.shutdown()

    def handle_feedback_update(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {round(feedback.progress, 2)}% complete.')


def main(args=None):
    rclpy.init(args=args)
    navigation_client = NavigationActionClient()
    rclpy.spin(navigation_client)

if __name__ == '__main__':
    main()
