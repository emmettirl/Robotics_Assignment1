#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robotics_assignment1.action import Turtle1Follow

class MoveTurtle1ActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Turtle1Follow,
            't1Follow',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        goal_turtle = goal_handle.request.goal_turtle_name

        goal_handle.succeed()
        result = Turtle1Follow.Result()

        self.get_logger().info('Goal: %s' % goal_turtle)
        result.distance = [100]
        return result


def main(args=None):
    rclpy.init(args=args)

    move_turtle1_action_server = MoveTurtle1ActionServer()

    rclpy.spin(move_turtle1_action_server)


if __name__ == '__main__':
    main()