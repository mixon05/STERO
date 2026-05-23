import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from my_interfaces.action import Interface
from copy import deepcopy
import math
import time


def calculate_path_distance(path):
    return sum(
        math.sqrt(
            (path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x) ** 2 +
            (path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y) ** 2
        )
        for i in range(1, len(path.poses))
    )

def convert_euler_to_quaternion(roll_angle, pitch_angle, yaw_angle):
    half_yaw_cos, half_yaw_sin = math.cos(yaw_angle * 0.5), math.sin(yaw_angle * 0.5)
    half_pitch_cos, half_pitch_sin = math.cos(pitch_angle * 0.5), math.sin(pitch_angle * 0.5)
    half_roll_cos, half_roll_sin = math.cos(roll_angle * 0.5), math.sin(roll_angle * 0.5)

    quaternion_x = half_roll_sin * half_pitch_cos * half_yaw_cos - half_roll_cos * half_pitch_sin * half_yaw_sin
    quaternion_y = half_roll_cos * half_pitch_sin * half_yaw_cos + half_roll_sin * half_pitch_cos * half_yaw_sin
    quaternion_z = half_roll_cos * half_pitch_cos * half_yaw_sin - half_roll_sin * half_pitch_sin * half_yaw_cos
    quaternion_w = half_roll_cos * half_pitch_cos * half_yaw_cos + half_roll_sin * half_pitch_sin * half_yaw_sin

    return (quaternion_x, quaternion_y, quaternion_z, quaternion_w)

class NavigationActionServer(Node):

    def __init__(self):
        super().__init__('navigation_action_server')

        self.action_server = ActionServer(
            self, Interface, 'navigate_action', self.handle_goal_execution
        )

        self.odom_subscription = self.create_subscription(
            Odometry, '/ground_truth_odom', self.process_odometry, 10
        )

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 'head_controller/joint_trajectory', 10
        )

        self.navigator = BasicNavigator()
        self.reset_state()

    def reset_state(self):
        self.current_pose = None
        self.init_pose = None
        self.current_z_twist = None
        self.routes = []
        self.rideDist = 0
        self.is_active = False

    def process_odometry(self, msg):
        if self.init_pose and self.is_active:
            self.rideDist += math.sqrt(
                (msg.pose.pose.position.x - self.current_pose.pose.position.x) ** 2 +
                (msg.pose.pose.position.y - self.current_pose.pose.position.y) ** 2
            )

        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = 'map'
        self.current_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.current_pose.pose = msg.pose.pose

        if not self.init_pose:
            self.init_pose = deepcopy(self.current_pose)

        self.current_z_twist = msg.twist.twist.angular.z
        self.publish_head_trajectory()

    def publish_head_trajectory(self):
        angle = max(min(self.current_z_twist, math.pi), -math.pi)
        trajectory_msg = JointTrajectory(
            joint_names=['head_1_joint', 'head_2_joint'],
            points=[JointTrajectoryPoint(
                positions=[angle, 0.0],
                time_from_start=Duration(seconds=0.2).to_msg()
            )]
        )
        self.trajectory_publisher.publish(trajectory_msg)

    def handle_goal_execution(self, goal_handle):
        self.is_active = True
        self.rideDist = 0
        self.get_logger().info('Starting navigation task')

        result = Interface.Result()

        if not goal_handle.request.route:
            result.result = 1
            self.is_active = False
            return result

        if not self.init_pose:
            result.result = 2
            self.is_active = False
            return result

        self.navigator.setInitialPose(self.init_pose)
        self.navigator.waitUntilNav2Active()

        self.setup_navigation_route(goal_handle.request.route)

        pathPoses = self.navigator.getPathThroughPoses(start=self.init_pose, goals=self.routes)
        pathDist = calculate_path_distance(pathPoses)
        self.get_logger().info(f'Total Path Length: {pathDist}')

        self.navigator.followWaypoints(self.routes)

        feedback = Interface.Feedback()
        while not self.navigator.isTaskComplete():
            nav_feedback = self.navigator.getFeedback()
            if nav_feedback:
                feedback.feedback = round((self.rideDist / pathDist) * 100, 2)
                self.get_logger().info(f'Progress: {feedback.feedback}%')
                goal_handle.publish_feedback(feedback)
                time.sleep(1.5)

        return self.finalize_task(goal_handle)

    def setup_navigation_route(self, route):
        for i in range(0, len(route), 3):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = route[i]
            pose.pose.position.y = route[i + 1]
            qx, qy, qz, qw = convert_euler_to_quaternion(0, 0, route[i + 2])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.routes.append(deepcopy(pose))

    def finalize_task(self, goal_handle):
        task_result = self.navigator.getResult()
        result = Interface.Result()

        if task_result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation task completed successfully')
            goal_handle.succeed()
            result.result = 0
        elif task_result == TaskResult.CANCELED:
            self.get_logger().info('Navigation task canceled')
            result.result = 3
        elif task_result == TaskResult.FAILED:
            self.get_logger().info('Navigation task failed')
            result.result = 4
        else:
            self.get_logger().info('Unknown result status')
            result.result = 5

        self.reset_state()
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = NavigationActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()
