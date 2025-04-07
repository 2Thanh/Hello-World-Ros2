import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__('joint_position_publisher')
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'joint_positions',
            10
        )
        # Define publishing frequency (Hz)
        self.publish_rate = 50.0  # 50Hz for smooth motion
        self.timer_period = 1.0 / self.publish_rate
        # Create a timer to publish at regular intervals
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize trajectory variables
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.trajectory_points = []
        self.trajectory_index = 0
        self.is_moving = False
        
        self.get_logger().info('Joint Position Publisher started')
    
    def timer_callback(self):
        if self.is_moving and self.trajectory_index < len(self.trajectory_points):
            # Get the next point in the trajectory
            next_position = self.trajectory_points[self.trajectory_index]
            # Publish it
            self.publish_positions(next_position)
            # Move to the next point
            self.trajectory_index += 1
            
            # Check if we've reached the end of the trajectory
            if self.trajectory_index >= len(self.trajectory_points):
                self.is_moving = False
                self.get_logger().info('Reached target position')
    
    def publish_positions(self, positions):
        msg = Float32MultiArray()
        msg.data = positions
        self.publisher_.publish(msg)
        self.current_positions = positions
    
    def generate_trajectory(self, start_positions, end_positions, duration):
        """
        Generate a smooth trajectory between start and end positions.
        
        Parameters:
        start_positions -- starting joint positions
        end_positions -- target joint positions
        duration -- time to complete the motion in seconds
        """
        # Calculate how many points we need based on the duration and publish rate
        num_points = int(duration * self.publish_rate)
        
        # Create empty trajectory array
        trajectory = []
        
        # Generate a smooth trajectory using linear interpolation
        for i in range(num_points):
            t = i / (num_points - 1)  # Normalized time (0 to 1)
            
            # Apply smoothing using sine acceleration profile (smoother start/stop)
            # This creates an S-curve velocity profile
            if t < 0.5:
                # Accelerating phase
                blend = 0.5 * (1 - np.cos(t * np.pi))
            else:
                # Decelerating phase
                blend = 0.5 * (1 + np.cos((t-1) * np.pi))
            
            # Interpolate each joint
            point = []
            for j in range(len(start_positions)):
                # Linear interpolation with smoothing
                value = start_positions[j] + blend * (end_positions[j] - start_positions[j])
                point.append(value)
            
            trajectory.append(point)
        
        return trajectory
    
    def move_to_position(self, target_positions, duration=1.0):
        """
        Move to a target position over a specified duration.
        
        Parameters:
        target_positions -- target joint positions
        duration -- time to complete the motion in seconds
        """
        # Generate trajectory from current position to target
        self.trajectory_points = self.generate_trajectory(
            self.current_positions, target_positions, duration
        )
        self.target_positions = target_positions
        self.trajectory_index = 0
        self.is_moving = True
        self.get_logger().info(f'Moving to position: {target_positions} over {duration} seconds')
        
        # Wait until motion is complete
        while self.is_moving and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.001)

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()
    
    try:
        # Example: send a sequence of joint positions
        positions_to_send = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0, 1.0, 0.0, 0.0],
            [0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
            [-0.5, -0.5, 0.0, 0.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        
        for positions in positions_to_send:
            # Move to each position with a smooth trajectory over 2 seconds
            node.move_to_position(positions, duration=2.0)
            # Optional pause at the target position
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()