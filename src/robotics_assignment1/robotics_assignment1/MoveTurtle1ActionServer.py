#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from robotics_assignment1.action import Turtle1Follow
from rclpy.task import Future
import math


class MoveTurtle1ActionServer(Node):
    turtles = {}
    controlled_turtle = 'turtle1'
    goal_turtle = None
    pose_subscribers = []

    def __init__(self):
        super().__init__(f'move_{self.controlled_turtle}_action_server')

        self.pose_subscribers
        for i in range(1, 10):
            turtle_name = f'turtle_{i}'
            subscriber = self.create_subscription(
                Pose,
                f'/{turtle_name}/pose',
                lambda msg, name=turtle_name: self.pose_callback(msg, name),
                10)
            self.pose_subscribers.append(subscriber)


        # also create a subscriber for controlled_turtle
        subscriber = self.create_subscription(
            Pose,
            f'/{self.controlled_turtle}/pose',
            lambda msg, name=f'{self.controlled_turtle}': self.pose_callback(msg, name),
            10)

        self._action_server = ActionServer(
            self,
            Turtle1Follow,
            't1Follow',
            self.execute_callback)

        # create a publisher for controlled turtle
        self.velocity_publisher = self.create_publisher(Twist, f'/{self.controlled_turtle}/cmd_vel', 10)

        self.reset_state()


    def pose_callback(self, msg, turtle_name):
        self.turtles[turtle_name] = msg

    def publish_twist(self, linear_speed, angular_speed):
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.velocity_publisher.publish(twist)

    def calculate_distance(self, pose1, pose2):
        return ((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)**0.5

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def set_linear_and_angular_speed(self, controlled_pose, goal_pose):

        linear_gain = 2
        angular_gain = 4

        delta_x = goal_pose.x - controlled_pose.x
        delta_y = goal_pose.y - controlled_pose.y
        distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

        angle_to_goal = math.atan2(delta_y, delta_x)

        # Calculate the difference in angle
        angle_diff = self.normalize_angle(angle_to_goal - controlled_pose.theta)

        # Set linear and angular speeds
        linear_speed = linear_gain* distance
        angular_speed = angular_gain * angle_diff

        self.publish_twist(linear_speed, angular_speed)


    def update_velocity(self):
        if self.controlled_turtle in self.turtles and self.goal_turtle in self.turtles:
            controlled_pose = self.turtles[self.controlled_turtle]
            goal_pose = self.turtles[self.goal_turtle]
            self.set_linear_and_angular_speed(controlled_pose, goal_pose)

            self.distance = self.calculate_distance(controlled_pose, goal_pose)

            feedback = Turtle1Follow.Feedback()
            feedback.partial_distance = [self.distance]
            self.goal_handle.publish_feedback(feedback)

            if self.distance <= 0.5:
                self.get_logger().info('Goal reached!')
                self.goal_handle.succeed()
                result = Turtle1Follow.Result()
                result.distance = [self.distance]
                self.get_logger().info(f'Distance: {result.distance}')

                self.publish_twist(0.0, 0.0)

                self.timer.cancel()
                self.result_future.set_result(result)
                self.reset_state()

            else:
                self.get_logger().info(f'Distance: {self.distance}')
        else:
            self.get_logger().warn('Controlled turtle or goal turtle not found in turtles dictionary.')
            self.goal_handle.abort()
            result = Turtle1Follow.Result()
            result.distance = [self.distance]
            self.result_future.set_result(result)
            self.reset_state()

    def execute_callback(self, goal_handle):
        self.get_logger().info('\n\nExecuting goal...')
        self.goal_handle = goal_handle
        self.result_future = Future()

        for turtle_name, pose in self.turtles.items():
            self.get_logger().info(f'{turtle_name}: {pose}\n')

        self.goal_turtle = self.goal_handle.request.goal_turtle_name

        if self.goal_turtle not in self.turtles:
            self.get_logger().warn(f'Goal turtle {self.goal_turtle} not found.')
            self.goal_handle.abort()
            result = Turtle1Follow.Result()
            result.distance = [0.0]
            return result

        # print target turtle information, the controlled turtle information, and the distance between them
        self.get_logger().info(f'Goal: {self.goal_turtle}, at: {self.turtles[self.goal_turtle]}')
        self.get_logger().info(f'Controlled Turtle: {self.controlled_turtle}: {self.turtles[self.controlled_turtle]}\n')

        self.distance = self.calculate_distance(self.turtles[self.goal_turtle], self.turtles[self.controlled_turtle])

        # create a timer to periodically update the turtle's velocity
        self.timer = self.create_timer(0.1, self.update_velocity)

        #wait for the goal to be completed
        rclpy.spin_until_future_complete(self, self.result_future)
        return self.result_future.result()

    def reset_state(self):
        self.goal_turtle = None
        self.distance = 0
        self.goal_handle = None
        self.result_future = None

def main(args=None):
    rclpy.init(args=args)

    move_turtle1_action_server = MoveTurtle1ActionServer()

    rclpy.spin(move_turtle1_action_server)


if __name__ == '__main__':
    main()