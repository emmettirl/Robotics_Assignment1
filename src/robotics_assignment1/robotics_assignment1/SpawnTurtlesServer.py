#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robotics_assignment1.srv import SpawnTurtles
from SpawnClient import SpawnClient
import math

class SpawnTurtlesServer(Node):
    def __init__(self):
        super().__init__('spawn_turtles_server')
        self.spawn_service = self.create_service(SpawnTurtles, 'spawn_turtles', self.handle_spawn_turtles_request)
        self.spawn_client = SpawnClient()
        self.client_futures = []

        self.get_logger().info('SpawnTurtles server ready.')

    def handle_spawn_turtles_request(self, request, response):
        self.get_logger().info(f'Received request to spawn {request.number_of_turtles} turtles.')
        n = request.number_of_turtles
        r = 4
        center_x, center_y = 5.5, 5.5
        turtle_names = []

        for i in range(n):
            self.get_logger().info(f'Spawning turtle {i + 1}...')
            theta = (2 * math.pi * i) / n
            x = r * math.cos(theta) + center_x
            y = r * math.sin(theta) + center_y
            orientation = theta + (math.pi / 2)
            name = f'turtle_{i + 1}'

            self.spawn_turtle(x, y, orientation, name)
            turtle_names.append(name)

        response.turtle_names = turtle_names
        self.get_logger().info(f'Spawned turtles: {", ".join(turtle_names)}')
        return response

    def spawn_turtle(self, x, y, theta, name):
        self.get_logger().info(f'Preparing to spawn turtle {name} at ({x}, {y}, {theta})')
        response = self.spawn_client.send_request(x, y, theta, name)
        if response:
            self.get_logger().info(f'Successfully spawned turtle: {response.name}')
            return True
        else:
            self.get_logger().error(f'Failed to spawn turtle: {name}')
            return False

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for future, name in self.client_futures:
                if future.done():
                    try:
                        response = future.result()
                        self.get_logger().info(f'Successfully spawned turtle: {response.name}')
                    except Exception as e:
                        self.get_logger().error(f'Service call failed for {name}: {e}')
                else:
                    incomplete_futures.append((future, name))
            self.client_futures = incomplete_futures

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtlesServer()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()