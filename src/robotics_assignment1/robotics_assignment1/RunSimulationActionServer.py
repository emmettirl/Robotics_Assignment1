#!/usr/bin/env python3

from robotics_assignment1.action import RunSimulation
import subprocess
import random
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading



class RunSimulationActionServer(Node):

    def __init__(self):
        super().__init__('run_commands_action_server')
        self._action_server = ActionServer(
            self,
            RunSimulation,
            'run_simulation',
            self.execute_callback)
        self.turtles_poses = {}
        self.turtle_list = []

        self.pose_subscribers = []
        for i in range(1, 10):
            turtle_name = f'turtle_{i}'
            subscriber = self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10)
            self.pose_subscribers.append(subscriber)

        self.velocity_publishers = {}
        for i in range(1, 10):
            turtle_name = f'turtle_{i}'
            publisher = self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
            self.velocity_publishers[turtle_name] = publisher

        self.spin_timer = self.create_timer(0.1, self.spin_turtles)


    def pose_callback(self, msg, turtle_name):
        self.turtles_poses[turtle_name] = msg


    def spin_turtles(self, angular_speed=0.5):
        radius = 4.0
        center_x = 5.5
        center_y = 5.5
        linear_speed = angular_speed * radius

        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        for turtle in self.turtle_list:
            if turtle in self.velocity_publishers:
                self.velocity_publishers[turtle].publish(twist)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        random_number = random.randint(3, 9)

        # Run the first command
        self.get_logger().info(f'Running: ros2 run robotics_assignment1 SpawnTurtlesServerClient.py {random_number}')
        subprocess.run(['ros2', 'run', 'robotics_assignment1', 'SpawnTurtlesServerClient.py', str(random_number)])

        for i in range(random_number):
            self.turtle_list.append(f'turtle_{i+1}')
        random.shuffle(self.turtle_list)

        # Create a new thread for running the action goals
        thread = threading.Thread(target=self.run_action_goals)
        thread.start()

        goal_handle.succeed()

        result = RunSimulation.Result()
        result.success = True
        return result

    def run_action_goals(self):
        for turtle in self.turtle_list:
            self.get_logger().info(f'Running: ros2 action send_goal t1Follow robotics_assignment1/action/Turtle1Follow goal_turtle_name: {turtle}')
            subprocess.Popen(['ros2', 'action', 'send_goal', 't1Follow', f'robotics_assignment1/action/Turtle1Follow', f'goal_turtle_name: {turtle}']).wait()


def main(args=None):
    rclpy.init(args=args)
    run_commands_action_server = RunSimulationActionServer()
    rclpy.spin(run_commands_action_server)

if __name__ == '__main__':
    main()