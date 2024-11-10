#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from robotics_assignment1.srv import SpawnTurtles


class SpawnTurtlesServerClient(Node):
    def __init__(self):
        super().__init__('spawn_turtles_client')
        self.client = self.create_client(SpawnTurtles, 'spawn_turtles')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SpawnTurtles service...')
        self.request = SpawnTurtles.Request()

    def send_request(self, number_of_turtles):
        self.request.number_of_turtles = number_of_turtles
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Received response from server')
            return self.future.result()
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)
    client = SpawnTurtlesServerClient()
    number_of_turtles = int(sys.argv[1])
    response = client.send_request(number_of_turtles)
    if response:
        client.get_logger().info(f'Spawned turtles: {", ".join(response.turtle_names)}')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
