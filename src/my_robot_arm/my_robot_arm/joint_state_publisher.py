import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create a subscriber for joint positions
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.joint_positions_callback,
            10)
            
        self.joint_names = ['hip', 'shoulder', 'elbow', 'wrist', 'l_g_base', 'r_g_base']
        self.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('Joint State Publisher started. Use "ros2 topic pub /joint_positions std_msgs/msg/Float32MultiArray ..." to set positions.')
        
        # Publish joint state periodically
        self.timer = self.create_timer(0.1, self.publish_joint_state)
    
    def joint_positions_callback(self, msg):
        # Check if the provided array length matches our joints
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(f'Expected {len(self.joint_names)} joint values, got {len(msg.data)}')
            return
            
        # Set the positions to the requested values
        self.positions = list(msg.data)
        self.get_logger().info(f'Received new positions: {self.positions}')
        
    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.positions
        msg.velocity = [0.0, 0.0]
        msg.effort = []
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()