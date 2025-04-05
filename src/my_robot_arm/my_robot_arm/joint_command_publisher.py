import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # Send every 2 seconds
        self.count = 0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2']
        # Alternate between two positions
        msg.position = [1.0, -1.0] if self.count % 2 == 0 else [-1.0, 1.0]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent joint command: {msg.position}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()