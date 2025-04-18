#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import message_filters
import tf2_ros
import tf2_geometry_msgs
import tf

class ColorTracker3DNode:
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
            ),
            #Green
            'green': (
                np.array([45, 100, 50]),  # lower_green 
                np.array([75, 255, 255]),  # upper_green
                (0, 255, 0)  # Display color (BGR): Green
            ),

            #Blue
            'blue': (
                np.array([94, 100, 50]),  # lower_blue
                np.array([130, 255, 255]),  # upper_blue
                (255, 0, 0)  # Display color (BGR): Blue
            ),
            #Red
            'red': (
                np.array([0, 150, 10]),  # lower_red
                np.array([35, 255, 255]),  # upper_red
                (255, 0, 0)  # Display color (BGR): Red
            )
        }
        
        # Initialize ROS publishers for each color's 3D centroids
        self.centroid_publishers = {}
        for color_name in self.color_bounds.keys():
            self.centroid_publishers[color_name] = rospy.Publisher(
                f'/color_tracker/{color_name}_centroid_3d', 
                PointStamped, 
                queue_size=10
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
        
        # Subscribe to camera info to get intrinsics
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        self.camera_info = None
        
        # TF2 Buffer for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Wait for camera info
        rospy.loginfo("Waiting for camera info...")
        while not self.camera_info and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Use synchronized subscribers for color and depth images
        self.color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        
        # Create synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        
        rospy.loginfo("Color tracker 3D node initialized successfully")
    
    def camera_info_callback(self, data):
        if self.camera_info is None:
            self.camera_info = data
            rospy.loginfo("Received camera info")
    
    def image_callback(self, color_msg, depth_msg):
        try:
            # Convert ROS Image messages to OpenCV images
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")  # 16-bit depth image
            
            # Create a copy of the original frame for drawing
            display_frame = cv_color.copy()
            
            # Process each color
            for color_name, (lower_bound, upper_bound, display_color) in self.color_bounds.items():
                # Segment the desired color
                mask, result = self.segment_color(cv_color, lower_bound, upper_bound)
                
                # Calculate centroids
                centroids = self.calculate_multiple_centroids(mask)
                
                # Publish mask for debugging
                try:
                    mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
                    self.mask_pubs[color_name].publish(mask_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"Failed to publish mask: {e}")
                
                # Draw centroids and publish their 3D coordinates
                if centroids:
                    for c in centroids:
                        # Draw circle at centroid
                        cv2.circle(display_frame, c, 5, display_color, -1)
                        
                        # Get depth at centroid
                        # Note: Need to handle potential alignment issues between color and depth
                        depth_mm = cv_depth[c[1], c[0]]  # Depth in mm at the centroid
                        
                        # Skip if depth is 0 (no valid depth data)
                        if depth_mm == 0:
                            continue
                        
                        # Convert to meters
                        depth_m = float(depth_mm) / 1000.0
                        
                        # Calculate 3D point using camera intrinsics
                        x_3d, y_3d, z_3d = self.pixel_to_3d(c[0], c[1], depth_m)
                        
                        # Add text with coordinates and color name
                        text = f"{color_name}: ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f})"
                        cv2.putText(display_frame, text, (c[0] + 10, c[1]), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, display_color, 2)
                        
                        # Publish 3D centroid as ROS message
                        point_msg = PointStamped()
                        point_msg.header = Header()
                        point_msg.header.stamp = rospy.Time.now()
                        point_msg.header.frame_id = "camera_color_optical_frame"  # Use appropriate frame
                        point_msg.point.x = x_3d
                        point_msg.point.y = y_3d
                        point_msg.point.z = z_3d
                        
                        try:
                            # Optional: Transform to a different frame (e.g., base frame)
                            # point_msg = self.transform_point(point_msg, "base_link")
                            
                            self.centroid_publishers[color_name].publish(point_msg)
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                                tf2_ros.ExtrapolationException) as e:
                            rospy.logwarn(f"TF error: {e}")
            
            # Publish the debug image
            try:
                debug_img_msg = self.bridge.cv2_to_imgmsg(display_frame, "bgr8")
                self.debug_image_pub.publish(debug_img_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to publish debug image: {e}")
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def pixel_to_3d(self, u, v, depth):
        """
        Convert a pixel coordinate and depth value to a 3D point in camera frame
        
        Args:
            u, v: Pixel coordinates
            depth: Depth value in meters
            
        Returns:
            (x, y, z): 3D coordinates in camera frame (meters)
        """
        # Get camera intrinsics from CameraInfo
        fx = self.camera_info.K[0]  # Focal length x
        fy = self.camera_info.K[4]  # Focal length y
        cx = self.camera_info.K[2]  # Principal point x
        cy = self.camera_info.K[5]  # Principal point y
        
        # Back-project pixel to 3D
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        

        ## Transform the point from camera frame to end effector frame (hardcoded)
        # translation = [0,-0.04,-0.10]
        # rotation_quat = [0,0,0,1]
        
        translation = [0,0,-0.03]
        rotation_quat = [0,0,0,1]
        

        q = np.array(rotation_quat)

        # Create the translation vector (x, y, z)
        t = np.array(translation)

        # Convert quaternion to a rotation matrix using tf.transformations
        rotation_matrix = tf.transformations.quaternion_matrix(q)[:3, :3]
        
        camera_point = np.array([x, y, z])
        
        end_effector_point = np.dot(rotation_matrix, camera_point) + t
        
        # print(f"end_effector_point: {end_effector_point}")
        # print(f"xyz point: {x}, {y}, {z}")
        
        return (end_effector_point[0], end_effector_point[1], end_effector_point[2])
        
        # return x,y,z
    
    def transform_point(self, point_stamped, target_frame):
        """
        Transform point from source frame to target frame
        
        Args:
            point_stamped: PointStamped in source frame
            target_frame: Target frame id
            
        Returns:
            Transformed PointStamped
        """
        return self.tf_buffer.transform(point_stamped, target_frame, rospy.Duration(1.0))
    
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
        color_tracker = ColorTracker3DNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()