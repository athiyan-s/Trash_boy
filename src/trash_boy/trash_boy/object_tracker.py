#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorTrackerNode(Node):
    def __init__(self):
        super().__init__('color_tracker_node')
        
        # Publisher for object position
        self.position_publisher = self.create_publisher(
            Point, 
            '/object_position', 
            10
        )
        
        # Publisher for visualization image (optional)
        self.image_publisher = self.create_publisher(
            Image,
            '/color_tracker/image',
            10
        )
        
        # CV Bridge for converting between ROS and OpenCV images
        self.bridge = CvBridge()
        
        # Global variables for color tracking
        self.target_color = None
        self.tracking = False
        
        # Open camera
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return
        
        # Get frame dimensions
        ret, test_frame = self.cap.read()
        if ret:
            self.frame_height, self.frame_width = test_frame.shape[:2]
            self.center_x = self.frame_width // 2
            self.center_y = self.frame_height // 2
            self.get_logger().info(f'Frame size: {self.frame_width}x{self.frame_height}')
            self.get_logger().info(f'Center point: ({self.center_x}, {self.center_y})')
        
        # Timer to process frames (30 Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
        self.get_logger().info('Color Tracker Node started!')
        self.get_logger().info('Click on an object in the video window to track it')
        self.get_logger().info('Coordinate system: (0,0) is at frame center')
        
    def pick_color(self, event, x, y, flags, param):
        """Mouse callback to select color to track"""
        if event == cv2.EVENT_LBUTTONDOWN:
            frame = param
            self.target_color = frame[y, x].tolist()
            self.tracking = True
            self.get_logger().info(f'Tracking color: {self.target_color}')
    
    def process_frame(self):
        """Process camera frame and detect object"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return
        
        # Flip frame
        frame = cv2.flip(frame, 1)
        
        # Store original frame for color picking
        original_frame = frame.copy()
        
        # Get frame dimensions
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        # Show instructions
        cv2.putText(frame, "Click on an object to track its color", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        object_detected = False
        obj_x_centered = 0
        obj_y_centered = 0
        
        if self.tracking and self.target_color is not None:
            # Convert to HSV for better color segmentation
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Convert clicked BGR color to HSV
            color_bgr = np.uint8([[self.target_color]])
            color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]
            
            # Define HSV range for detection
            lower = np.array([max(0, color_hsv[0] - 15), 80, 80])
            upper = np.array([min(179, color_hsv[0] + 15), 255, 255])
            
            # Create mask
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find largest contour (main object)
            largest_contour = None
            max_area = 0
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # Ignore small noise
                    if area > max_area:
                        max_area = area
                        largest_contour = cnt
            
            # If object found, draw and publish
            if largest_contour is not None:
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center of bounding box (in pixel coordinates)
                obj_x_pixel = x + w // 2
                obj_y_pixel = y + h // 2
                
                # Convert to centered coordinate system
                # X: positive to the right, negative to the left
                # Y: positive downward, negative upward (standard image coordinates)
                # If you want Y positive upward, use: obj_y_centered = center_y - obj_y_pixel
                obj_x_centered = obj_x_pixel - center_x
                obj_y_centered = obj_y_pixel - center_y
                
                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(frame, (obj_x_pixel, obj_y_pixel), 5, (0, 0, 255), -1)
                
                # Draw line from center to object
                cv2.line(frame, (center_x, center_y), (obj_x_pixel, obj_y_pixel), (255, 0, 255), 2)
                
                # Show centered coordinates
                cv2.putText(frame, f"X: {obj_x_centered}, Y: {obj_y_centered}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                object_detected = True
                
                # Publish position (centered coordinates)
                position_msg = Point()
                position_msg.x = float(obj_x_centered)
                position_msg.y = float(obj_y_centered)
                position_msg.z = 0.0  # Not used, but required for Point message
                
                self.position_publisher.publish(position_msg)
                
                # Log periodically (every 30 frames ~ 1 second)
                if hasattr(self, 'frame_count'):
                    self.frame_count += 1
                else:
                    self.frame_count = 0
                
                if self.frame_count % 30 == 0:
                    self.get_logger().info(f'Object position (centered): x={obj_x_centered}, y={obj_y_centered}')
        
        # Show tracking status
        status_text = "Tracking: ON" if self.tracking else "Tracking: OFF"
        status_color = (0, 255, 0) if object_detected else (0, 0, 255)
        cv2.putText(frame, status_text, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Draw coordinate system AFTER all object detection (so it doesn't interfere)
        # Vertical line (Y-axis) - semi-transparent by drawing thinner
        cv2.line(frame, (center_x, 0), (center_x, height), (128, 128, 128), 1, cv2.LINE_AA)
        # Horizontal line (X-axis)
        cv2.line(frame, (0, center_y), (width, center_y), (128, 128, 128), 1, cv2.LINE_AA)
        # Center point
        cv2.circle(frame, (center_x, center_y), 3, (255, 255, 0), -1)
        cv2.putText(frame, "(0,0)", (center_x + 10, center_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Show video feed
        cv2.imshow("Color Tracker - ROS2", frame)
        
        # Mouse callback - use ORIGINAL frame without axes for color picking
        cv2.setMouseCallback("Color Tracker - ROS2", self.pick_color, param=original_frame)
        
        # Publish visualization image (optional)
        try:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish image: {str(e)}')
        
        # Handle key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()
        elif key == ord('r'):
            # Reset tracking
            self.tracking = False
            self.target_color = None
            self.get_logger().info('Tracking reset. Click to select new color.')
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = ColorTrackerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()