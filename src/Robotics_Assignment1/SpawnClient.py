import sys
from std_msgs.msg import Int32
import rclpy
from rclpy.node import Node

class SpawnClient(Node):
    def __init__(self):
        super().__init__('spawn_client')
        self.pub = self.create_publisher(Int32, 'number_of_turtles', 10)

    def send_request(self, number_of_turtles):
        msg = Int32()
        msg.data = number_of_turtles
        self.pub.publish(msg)
        self.get_logger().info('Published number of turtles: %d' % number_of_turtles)

def main(args=None):
    rclpy.init(args=args)
    spawn_client = SpawnClient()
    number_of_turtles = int(sys.argv[1])
    spawn_client.send_request(number_of_turtles)
    spawn_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()