#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math


class ObjectFollowerNode(Node):
    def __init__(self):
        super().__init__('object_follower_node')
        
        # Subscriber to object position
        self.position_subscriber = self.create_subscription(
            Point,
            '/object_position',
            self.position_callback,
            10
        )
        
        # Publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Control parameters
        self.declare_parameter('kp_x', 0.005)  # Proportional gain for X (strafe) - INCREASED
        self.declare_parameter('kp_y', 0.005)  # Proportional gain for Y (forward/backward) - INCREASED
        self.declare_parameter('kd_x', 0.001)  # Derivative gain for X (damping)
        self.declare_parameter('kd_y', 0.001)  # Derivative gain for Y (damping)
        self.declare_parameter('max_linear_speed', 1.0)  # Max linear speed (m/s) - INCREASED
        self.declare_parameter('min_speed_threshold', 0.05)  # Minimum speed to move
        self.declare_parameter('deadzone', 30.0)  # Deadzone in pixels (object is "centered")
        self.declare_parameter('enable_following', True)  # Enable/disable following
        
        # Get parameters
        self.kp_x = self.get_parameter('kp_x').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kd_x = self.get_parameter('kd_x').value
        self.kd_y = self.get_parameter('kd_y').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_speed_threshold = self.get_parameter('min_speed_threshold').value
        self.deadzone = self.get_parameter('deadzone').value
        self.enable_following = self.get_parameter('enable_following').value
        
        # State variables
        self.last_seen_time = self.get_clock().now()
        self.object_visible = False
        self.timeout = 1.0  # seconds - stop if object not seen for this long
        
        # Previous error for derivative control
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = self.get_clock().now()
        
        # Timer for safety check (10 Hz)
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Object Follower Node started!')
        self.get_logger().info(f'Parameters: kp_x={self.kp_x}, kp_y={self.kp_y}')
        self.get_logger().info(f'Max speed: {self.max_linear_speed} m/s, Deadzone: {self.deadzone} px')
        self.get_logger().info('Waiting for object position...')
        
    def position_callback(self, msg):
        """Callback for object position - controls robot to keep object centered"""
        if not self.enable_following:
            return
        
        # Update last seen time
        self.last_seen_time = self.get_clock().now()
        self.object_visible = True
        
        # Get object position (centered coordinates)
        obj_x = msg.x  # Positive = right, Negative = left
        obj_y = msg.y  # Positive = down (away), Negative = up (closer)
        
        # Create velocity command
        twist = Twist()
        
        # Check if object is within deadzone (considered "centered")
        distance_from_center = math.sqrt(obj_x**2 + obj_y**2)
        
        if distance_from_center < self.deadzone:
            # Object is centered - stop moving
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            
            if hasattr(self, 'centered_logged') and not self.centered_logged:
                self.get_logger().info('Object centered! Robot stopped.')
                self.centered_logged = True
        else:
            self.centered_logged = False
            
            # Proportional control
            # Y-axis: Forward/Backward movement
            # Negative obj_y means object is above center (closer) -> move backward
            # Positive obj_y means object is below center (farther) -> move forward
            vel_x = self.kp_y * obj_y
            
            # X-axis: Strafe left/right
            # Positive obj_x means object is right of center -> strafe right
            # Negative obj_x means object is left of center -> strafe left
            vel_y = -self.kp_x * obj_x  # Negative because robot's Y is opposite
            
            # Limit speeds
            vel_x = max(-self.max_linear_speed, min(self.max_linear_speed, vel_x))
            vel_y = max(-self.max_linear_speed, min(self.max_linear_speed, vel_y))
            
            # Set velocity commands
            twist.linear.x = vel_x
            twist.linear.y = vel_y
            twist.angular.z = 0.0  # No rotation for now
            
            # Log periodically
            if hasattr(self, 'log_counter'):
                self.log_counter += 1
            else:
                self.log_counter = 0
            
            if self.log_counter % 30 == 0:  # Log every ~1 second
                self.get_logger().info(
                    f'Object at ({obj_x:.0f}, {obj_y:.0f}) | '
                    f'Vel: X={vel_x:.2f}, Y={vel_y:.2f} | '
                    f'Dist: {distance_from_center:.0f}px'
                )
        
        # Publish velocity command
        self.cmd_vel_publisher.publish(twist)
    
    def safety_check(self):
        """Safety timer - stop robot if object not seen for timeout period"""
        if not self.object_visible:
            return
        
        current_time = self.get_clock().now()
        time_since_last_seen = (current_time - self.last_seen_time).nanoseconds / 1e9
        
        if time_since_last_seen > self.timeout:
            # Object lost - stop robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            
            if self.object_visible:  # Log only once
                self.get_logger().warn('Object lost! Stopping robot.')
                self.object_visible = False


def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on exit
        twist = Twist()
        node.cmd_vel_publisher.publish(twist)
        node.get_logger().info('Shutting down - robot stopped')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()