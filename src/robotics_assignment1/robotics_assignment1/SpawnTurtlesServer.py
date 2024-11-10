#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from robotics_assignment1.srv import SpawnTurtles
import math

class SpawnTurtlesServer(Node):
    def __init__(self):
        super().__init__('spawn_turtles_server')
        self.spawn_service = self.create_service(SpawnTurtles, 'spawn_turtles', self.handle_spawn_turtles_request)
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('SpawnTurtles server ready.')
        self.client_futures = []

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

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        self.get_logger().info(f'Sending spawn request for {name}')
        self.get_logger().info(f'x: {x}, y: {y}, theta: {theta}')

        future = self.client.call_async(request)
        self.client_futures.append((future, name))

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


#
# import time
# import rclpy
# from rclpy.node import Node
# from turtlesim.srv import Spawn
# from robotics_assignment1.srv import SpawnTurtles
# from custom_interfaces.srv import CircleDuration
# from geometry_msgs.msg import Twist
# import math
#
#
# class SpawnTurtlesServer(Node):
#     def __init__(self):
#         super().__init__('spawn_turtles_server')
#         self.spawn_service = self.create_service(SpawnTurtles, 'spawn_turtles', self.handle_spawn_turtles_request)
#
#
#
#         self.client = self.create_client(Spawn, '/spawn')
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')
#         self.get_logger().info('SpawnTurtles server ready.')
#
#     def handle_spawn_turtles_request(self, request, response):
#         self.get_logger().info(f'Received request to spawn {request.number_of_turtles} turtles.')
#         n = request.number_of_turtles
#         r = 4
#         center_x, center_y = 5.5, 5.5
#         turtle_names = []
#
#         for i in range(n):
#             self.get_logger().info(f'Spawning turtle {i + 1}...')
#             theta = (2 * math.pi * i) / n
#             x = r * math.cos(theta) + center_x
#             y = r * math.sin(theta) + center_y
#             orientation = theta + (math.pi / 2)
#             name = f'turtle_{i + 1}'
#
#             if self.spawn_turtle(x, y, orientation, name):
#                 self.get_logger().info(f'Spawned {name} at ({x}, {y}, {orientation})')
#                 turtle_names.append(name)
#             else:
#                 self.get_logger().error(f'Failed to spawn {name}')
#
#         response.turtle_names = turtle_names
#         self.get_logger().info(f'Spawned turtles: {", ".join(turtle_names)}')
#         return response
#
#     def spawn_turtle(self, x, y, theta, name):
#         self.get_logger().info(f'Preparing to spawn turtle {name} at ({x}, {y}, {theta})')
#
#         request = Spawn.Request()
#         request.x = x
#         request.y = y
#         request.theta = theta
#         request.name = name
#
#         # log the request and params
#         self.get_logger().info(f'Sending spawn request for {name}')
#         self.get_logger().info(f'x: {x}, y: {y}, theta: {theta}')
#
#         future = self.client.call_async(request)
#         self.get_logger().info(f'Waiting for spawn response for {name}')
#
#         rclpy.spin_until_future_complete(self, future)
#         if future.done():
#             try:
#                 response = future.result()
#                 self.get_logger().info(f'Successfully spawned turtle: {response.name}')
#                 return response.name
#             except Exception as e:
#                 self.get_logger().error(f'Service call failed: {e}')
#                 return None
#         else:
#             self.get_logger().error('Service call did not complete')
#             return None
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = SpawnTurtlesServer()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == '__main__':
#     main()
