import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray, 
            'joint_positions', 
            10
        )
        self.get_logger().info('Joint Position Publisher started')
        
    def send_joint_positions(self, positions):
        msg = Float32MultiArray()
        msg.data = positions
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint positions: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()
    
    try:
        # Example: send a sequence of joint positions
        positions_to_send = [
            [0.0, 0.0],
            [0.5, 0.5],
            [1.0, 0.0],
            [0.5, -0.5],
            [0.0, -1.0],
            [-0.5, -0.5],
            [-1.0, 0.0],
            [-0.5, 0.5],
            [0.0, 0.0]
        ]
        
        for positions in positions_to_send:
            node.send_joint_positions(positions)
            time.sleep(2.0)  # Wait 2 seconds between positions
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()