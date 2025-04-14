#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

class ColorTrackerNode:
    def __init__(self):
        rospy.init_node('color_tracker_node', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Define color ranges with display colors in BGR format
        self.color_bounds = {
            # Orange
            'orange': (
                np.array([0, 180, 100]),  # lower_myorange
                np.array([35, 255, 255]),  # upper_myorange
                (0, 165, 255)  # Display color (BGR): Orange
            ),
            
            # Brown
            'brown': (
                np.array([10, 90, 20]),  # lower_brown 
                np.array([30, 255, 200]),  # upper_brown
                (42, 42, 165)  # Display color (BGR): Brown
            )
        }
        
        # Initialize ROS publishers for each color's centroids
        self.centroid_publishers = {}
        for color_name in self.color_bounds.keys():
            self.centroid_publishers[color_name] = rospy.Publisher(
                f'/color_tracker/{color_name}_centroid', 
                Point, 
                queue_size=10
            )
        
        # Subscribe to RealSense color image topic
        # Note: Adjust the topic name based on your RealSense ROS setup
        self.image_sub = rospy.Subscriber(
            '/camera/color/image_raw', 
            Image, 
            self.image_callback
        )
        
        # Optional: Create debug image publishers
        self.debug_image_pub = rospy.Publisher('/color_tracker/debug_image', Image, queue_size=10)
        self.mask_pubs = {}
        for color_name in self.color_bounds.keys():
            self.mask_pubs[color_name] = rospy.Publisher(
                f'/color_tracker/mask/{color_name}', 
                Image, 
                queue_size=10
            )
        
        rospy.loginfo("Color tracker node initialized successfully")
    
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Create a copy of the original frame for drawing
            display_frame = cv_image.copy()
            
            # Create a black image for combined results
            combined_result = np.zeros_like(cv_image)
            
            # Process each color
            for color_name, (lower_bound, upper_bound, display_color) in self.color_bounds.items():
                # Segment the desired color
                mask, result = self.segment_color(cv_image, lower_bound, upper_bound)
                
                # Calculate centroids
                centroids = self.calculate_multiple_centroids(mask)
                
                # Add this color's result to the combined result
                combined_result = cv2.add(combined_result, result)
                
                # Publish mask for debugging
                try:
                    mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
                    self.mask_pubs[color_name].publish(mask_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"Failed to publish mask: {e}")
                
                # Draw centroids and publish their coordinates
                if centroids:
                    for c in centroids:
                        # Draw circle at centroid
                        cv2.circle(display_frame, c, 5, display_color, -1)
                        
                        # Add text with coordinates and color name
                        text = f"{color_name}: ({c[0]}, {c[1]})"
                        cv2.putText(display_frame, text, (c[0] + 10, c[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, display_color, 2)
                        
                        # Also draw on the combined result
                        cv2.circle(combined_result, c, 5, display_color, -1)
                        cv2.putText(combined_result, text, (c[0] + 10, c[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, display_color, 2)
                        
                        # Publish centroid as ROS message
                        point_msg = Point()
                        point_msg.x = float(c[0])
                        point_msg.y = float(c[1])
                        point_msg.z = 0.0  # No depth information from RGB only
                        
                        self.centroid_publishers[color_name].publish(point_msg)
            
            # Publish the debug image
            try:
                debug_img_msg = self.bridge.cv2_to_imgmsg(display_frame, "bgr8")
                self.debug_image_pub.publish(debug_img_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to publish debug image: {e}")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def segment_color(self, frame, lower_bound, upper_bound):
        """
        Segment a specific color range from an image.
        
        Args:
            frame: Input image/video frame
            lower_bound: Lower HSV boundary for the color
            upper_bound: Upper HSV boundary for the color
        
        Returns:
            mask: Binary mask where the color is detected
            result: Original frame with only the segmented color visible
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask using the specified color range
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Apply the mask to the original image
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        return mask, result
    
    def calculate_multiple_centroids(self, mask):
        """
        Calculate centroids of multiple objects in a binary mask.
        
        Args:
            mask: Binary mask image
        
        Returns:
            centroids: List of (x, y) tuples containing the coordinates of each centroid
        """
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centroids = []
        
        # Process each contour
        for contour in contours:
            # Calculate area to filter out small noise contours
            area = cv2.contourArea(contour)
            
            # Filter by area (adjust the threshold as needed)
            if area > 4500:  # Minimum area threshold
                # Calculate moments for this contour
                M = cv2.moments(contour)
                
                # Calculate centroid
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
        
        return centroids

def main():
    try:
        color_tracker = ColorTrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
