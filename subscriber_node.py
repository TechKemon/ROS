# Now let's create a subscriber (listener)
# subscriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        # Give our node a name
        super().__init__('listener')
        
        # Create a subscriber:
        # - It will subscribe to String messages
        # - On the topic 'greetings'
        # - And call listener_callback when a message arrives
        self.subscription = self.create_subscription(
            String,
            'greetings',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # When we receive a message, log it
        self.get_logger().info(f'I heard: {msg.data}')


# How to run the subscriber:
def main_subscriber():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()