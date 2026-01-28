import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 1
        )
        
        # Publish lane info + debug image
        self.lane_info_pub = self.create_publisher(
            Float32MultiArray, "/lane_detection/lane_info", 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, "/lane_detection/debug_image", 10
        )
        self.get_logger().info('Lane Detector started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            lanes, debug_img = self.detect_lanes(cv_image)
            self.publish_lane_info(lanes)
            
            # Publish debug visualization
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def detect_lanes(self, image):
        """Detect yellow/white lanes using HSV + Hough"""
        debug_image = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Yellow lanes
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # White lanes  
        lower_white = np.array([0, 0, 100])
        upper_white = np.array([180, 30, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
        # Combine masks
        combined = cv2.bitwise_or(mask_yellow, mask_white)
        
        # Clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        
        # Edge detection
        edges = cv2.Canny(combined, 50, 150)
        
        # Find lane lines
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 
                               minLineLength=50, maxLineGap=10)
        
        lanes = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Normalized position (0=left, 1=right)
                mid_x = (x1 + x2) / 2
                norm_x = mid_x / image.shape[1]
                lanes.append(norm_x)
                
                # Draw line on debug image
                cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        
        # Draw center line
        cv2.line(debug_image, (image.shape[1]//2, 0), 
                (image.shape[1]//2, image.shape[0]), (0, 0, 255), 2)
        
        return lanes, debug_image

    def publish_lane_info(self, lanes):
        msg = Float32MultiArray()
        if lanes:
            msg.data = [float(len(lanes))] + [float(x) for x in lanes]
        else:
            msg.data = [0.0]
        self.lane_info_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
