import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from turtlesim.srv import Spawn
import math


class SpawnService(Node):
    def __init__(self):
        super().__init__('spawn_service')
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.spawn_client.wait_for_service()
        self.sub = self.create_subscription(Int32, 'number_of_turtles', self.spawn_turtles, 10)
        self.spawned_turtles = []

    def spawn_turtles(self, msg):
        n = msg.data
        r = 4
        center_x, center_y = 5.5, 5.5
        self.spawned_turtles.clear()

        for i in range(n):
            theta = (2 * math.pi * i) / n
            x = r * math.cos(theta) + center_x
            y = r * math.sin(theta) + center_y
            orientation = theta + (math.pi / 2)
            name = f'turtle_{i + 1}'

            self.spawn_turtle(x, y, orientation, name)

            self.get_logger().info(f'Scheduled spawn for {name} at x: {x}, y: {y}, theta: {orientation}')

    def spawn_turtle(self, x, y, theta, name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        self.get_logger().info(f'Spawning turtle {name} at ({x}, {y}, {theta})...')


        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda future: self.handle_spawn_response(future, name))

    def handle_spawn_response(self, future, name):
        try:
            response = future.result()
            self.spawned_turtles.append(response.name)
            self.get_logger().info(f'Turtle {response.name} spawned successfully!')
            if len(self.spawned_turtles) == int(self.spawned_turtles[-1].split('_')[-1]):
                self.get_logger().info(f"All turtles spawned: {', '.join(self.spawned_turtles)}")
        except Exception as e:
            self.get_logger().error(f'Failed to spawn turtle {name}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()