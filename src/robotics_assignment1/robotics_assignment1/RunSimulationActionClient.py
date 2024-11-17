#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robotics_assignment1.action import RunSimulation
import threading


class RunSimulationActionClient(Node):

    def __init__(self):
        super().__init__('run_simulation_action_client')
        self._action_client = ActionClient(self, RunSimulation, 'run_simulation')

    def send_goal(self):
        goal_msg = RunSimulation.Goal()
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        timer_thread = threading.Thread(target=self.start_timer)
        timer_thread.start()

    def start_timer(self):
        self.timer = self.create_timer(20.0, self.cancel_goal)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')

    def cancel_goal(self):
        self.get_logger().info('Cancelling goal due to timeout')
        self._action_client.cancel_goal_async(self._send_goal_future.result())

def main(args=None):
    rclpy.init(args=args)
    action_client = RunSimulationActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()