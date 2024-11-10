#!/usr/bin/env python3

import sys
from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node
class SpawnClient(Node):

    def __init__(self):
        super().__init__('spawn_client')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()

    def send_request(self, x, y, theta, name):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    spawn_client = SpawnClient()
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])
    name = sys.argv[4]
    response = spawn_client.send_request(x, y, theta, name)
    spawn_client.get_logger().info(
        'Result of spawn: %s' % response.name)
    spawn_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()