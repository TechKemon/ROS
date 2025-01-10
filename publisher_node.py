# First, let's create a simple publisher (talker)
# publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        # Give our node a name
        super().__init__('talker')
        
        # Create a publisher:
        # - It will publish String messages
        # - On the topic 'greetings'
        # - Queue size of 10 messages
        self.publisher = self.create_publisher(String, 'greetings', 10)
        
        # Create a timer that calls our function every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # Create a message
        msg = String()
        msg.data = f'Hello World: {self.count}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Let us know it was sent
        self.get_logger().info(f'Publishing: {msg.data}')
        self.count += 1

# How to run the publisher:
def main_publisher():
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
