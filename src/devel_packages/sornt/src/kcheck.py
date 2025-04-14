import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped
import tf.transformations as tft

def get_camera_to_base_transform(tf_listener=None):
    """
    Calculate the transformation from camera to robot base frame.
    
    Args:
        tf_listener: An existing tf listener, creates a new one if None
        
    Returns:
        numpy.ndarray: 4x4 transformation matrix from camera to base frame
    """
    # Create tf listener if not provided
    if tf_listener is None:
        tf_listener = tf.TransformListener()
        # Give time for the listener to get transforms
        rospy.sleep(1.0)
    
    try:
        # First get the transform from end effector to base
        tf_listener.waitForTransform("panda_link0", "panda_end_effector", rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = tf_listener.lookupTransform("panda_link0", "panda_end_effector", rospy.Time(0))
        
        # Convert to 4x4 transformation matrix
        ee_to_base_matrix = tft.concatenate_matrices(
            tft.translation_matrix(trans),
            tft.quaternion_matrix(rot)
        )
        
        # Define camera position relative to end effector
        # Camera is 9cm behind, 4cm above, and 5cm to the left of the gripper
        # Note: Assuming robot coordinate convention where:
        # X is forward, Y is left, Z is up from the gripper's perspective
        camera_to_ee_trans = [-0.09, 0.05, 0.04]  # [behind, left, above]
        
        # Identity rotation (assuming camera and gripper have same orientation)
        # If camera has different orientation, replace with proper rotation matrix
        camera_to_ee_rot = tft.quaternion_matrix([1, 0, 0, 0])  # Identity rotation
        camera_to_ee_rot[0:3, 3] = camera_to_ee_trans  # Add translation
        
        # Calculate camera to base transformation
        camera_to_base_matrix = np.dot(ee_to_base_matrix, camera_to_ee_rot)
        
        return camera_to_base_matrix
    
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"TF Error: {e}")
        return None

def transform_point_camera_to_base(point_in_camera, camera_to_base_matrix):
    """
    Transform a 3D point from camera frame to robot base frame.
    
    Args:
        point_in_camera: Point or [x, y, z] coordinates in camera frame
        camera_to_base_matrix: 4x4 transformation matrix from camera to base
        
    Returns:
        numpy.ndarray: [x, y, z] coordinates in robot base frame
    """
    # Convert point to homogeneous coordinates
    if isinstance(point_in_camera, Point):
        point_homogeneous = np.array([point_in_camera.x, point_in_camera.y, point_in_camera.z, 1.0])
    else:
        point_homogeneous = np.append(np.array(point_in_camera), 1.0)
    
    # Transform the point
    point_in_base_homogeneous = np.dot(camera_to_base_matrix, point_homogeneous)
    
    # Convert back from homogeneous to 3D coordinates
    point_in_base = point_in_base_homogeneous[:3] / point_in_base_homogeneous[3]
    
    return point_in_base

def create_point_in_base_frame(point_in_camera):
    """
    Create a PointStamped message for a point transformed from camera to base frame.
    
    Args:
        point_in_camera: Point or [x, y, z] coordinates in camera frame
        
    Returns:
        geometry_msgs.msg.PointStamped: Point message in base frame
    """
    # Get transformation matrix
    camera_to_base_matrix = get_camera_to_base_transform()
    
    if camera_to_base_matrix is None:
        rospy.logerr("Failed to get camera to base transformation")
        return None
    
    # Transform the point
    point_in_base = transform_point_camera_to_base(point_in_camera, camera_to_base_matrix)
    
    # Create message
    point_msg = PointStamped()
    point_msg.header.frame_id = "panda_link0"  # Base frame
    point_msg.header.stamp = rospy.Time.now()
    point_msg.point.x = point_in_base[0]
    point_msg.point.y = point_in_base[1]
    point_msg.point.z = point_in_base[2]
    
    return point_msg

# Example usage in your existing code
def process_realsense_point(realsense_point, tf_listener=None):
    """
    Process a point from RealSense camera and convert to robot base frame.
    
    Args:
        realsense_point: Point in camera frame
        tf_listener: Existing tf listener (optional)
        
    Returns:
        numpy.ndarray: Point coordinates in robot base frame
    """
    # Get camera to base transformation
    camera_to_base_matrix = get_camera_to_base_transform(tf_listener)
    
    if camera_to_base_matrix is None:
        rospy.logerr("Failed to get camera to base transformation")
        return None
    
    # Transform the point
    base_point = transform_point_camera_to_base(realsense_point, camera_to_base_matrix)
    
    return base_point

# Example integration with your existing get_value_for_orange_centroid method
def get_value_for_orange_centroid_in_base_frame(self):
    """
    Get orange centroid in robot base frame.
    
    Returns:
        geometry_msgs.msg.Point: Centroid coordinates in robot base frame
    """
    # First get the point in camera frame
    camera_point = self.get_value_for_orange_centroid()
    
    if camera_point is None:
        return None
        
    # Get transformation and convert to base frame
    camera_to_base_matrix = get_camera_to_base_transform()
    
    if camera_to_base_matrix is None:
        rospy.logerr("Failed to get camera to base transformation")
        return None
        
    # Transform the point
    base_coords = transform_point_camera_to_base(camera_point, camera_to_base_matrix)
    
    # Create a new Point message
    base_point = Point()
    base_point.x = base_coords[0]
    base_point.y = base_coords[1]
    base_point.z = base_coords[2]
    
    return base_point