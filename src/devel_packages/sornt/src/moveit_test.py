import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.sornt.src.moveit_class import MoveItPlanner
from geometry_msgs.msg import Pose
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import tf
import numpy as np
import time
from visualization_msgs.msg import Marker

from src.devel_packages.sornt.src.shelf import Shelf

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from autolab_core import RigidTransform


# This script plans a straight line path from the current robot pose to pose_goal
# This plan can then be executed on the robot using the execute_plan method

# create a MoveItPlanner object and start the moveit node
class Sort_MoveIt():
    def __init__(self, shelf: Shelf):
        self.franka_moveit = MoveItPlanner()
        # Intialize the environment with the shelf
        self.add_collision_boxes(shelf.shelf_as_collision_boxes())
    
    def move_to_second_home(self):
        # second_home, _ = self.franka_moveit.get_plan_given_joint([0.09446621,  0.23436419,  0.51143809, -2.13887694, -1.22993535,  1.01465016, 0.07993947])
        # self.franka_moveit.execute_plan(second_home)
        
        #KEERTHI
        joint_goal = [0.09446621,  0.23436419,  0.51143809, -2.13887694, -1.22993535,  1.01465016, 0.07993947]
        self.franka_moveit.goto_joint(joint_goal)
        
        
    def get_current_ee_pose(self):
        # Set up a tf listener
        listener = tf.TransformListener()
        
        # Wait for the transform to be available
        listener.waitForTransform("panda_link0", "panda_end_effector", rospy.Time(0), rospy.Duration(4.0))
        
        # Get the transform
        (trans, rot) = listener.lookupTransform("panda_link0", "panda_end_effector", rospy.Time(0))
        
        # Create a pose message
        current_pose = geometry_msgs.msg.Pose()
        current_pose.position.x = trans[0]
        current_pose.position.y = trans[1]
        current_pose.position.z = trans[2]
        current_pose.orientation.x = rot[0]
        current_pose.orientation.y = rot[1]
        current_pose.orientation.z = rot[2]
        current_pose.orientation.w = rot[3]
        
        return current_pose
    
    
    #KEERTHI
    def goto_pose(self, centroid):
        # Get current end effector pose
        ee_to_base_transform = self.franka_moveit.get_pose()
        print(f"Current End Effector Pose in Base Frame: {ee_to_base_transform}")
        
        # Based on the error and printed output, ee_to_base_transform is a RigidTransform object
        # with translation and rotation matrix components, not a Pose message with quaternions
        
        # Extract the translation and rotation directly from the RigidTransform
        ee_to_base = np.eye(4)
        
        # Copy the rotation matrix (3x3) to the upper left of the transformation matrix
        ee_to_base[0:3, 0:3] = ee_to_base_transform.rotation
        
        # Copy the translation vector to the right column
        ee_to_base[0:3, 3] = ee_to_base_transform.translation
        
        # Define camera-to-end-effector transformation
        camera_to_ee_translation = [-0.09, 0.05, 0.04]
        camera_to_ee = np.eye(4)
        camera_to_ee[0:3, 3] = camera_to_ee_translation
        
        # Calculate camera position in base frame
        camera_to_base = np.matmul(ee_to_base, camera_to_ee)
        
        # Transform point from camera frame to base frame
        point_in_camera = np.array([centroid.x, centroid.y, centroid.z, 1.0])
        point_in_base_frame = np.matmul(camera_to_base, point_in_camera)
        
        # Extract quaternion from the rotation matrix for output
        # Note: We need to get the quaternion components from the RigidTransform object
        # Assuming the quaternion is stored or can be accessed as shown in the printed output
        qx, qy, qz, qw = ee_to_base_transform.quaternion
        
        # Print the coordinates that should be given as input to goto_pose in frankapy
        print("=== Coordinates for frankapy goto_pose in base frame ===")
        print(f"Position X: {point_in_base_frame[0]:.4f}")
        print(f"Position Y: {point_in_base_frame[1]:.4f}")
        print(f"Position Z: {point_in_base_frame[2]:.4f}")
        print(f"Orientation X: {qx:.4f}")
        print(f"Orientation Y: {qy:.4f}")
        print(f"Orientation Z: {qz:.4f}")
        print(f"Orientation W: {qw:.4f}")
        print("=======================================")
        
        # Create a Point message with the transformed coordinates (for potential future use)
        target_point = geometry_msgs.msg.Point()
        target_point.x = point_in_base_frame[0]
        target_point.y = point_in_base_frame[1]
        target_point.z = point_in_base_frame[2]

        des_pose = RigidTransform(
        rotation=ee_to_base_transform.rotation,
        translation=[0.45209972, 0.04327647, 0.36306605],
        # translation=[0.54990976, -0.38635197, 0.55136056],
        # translation=np.array([point_in_base_frame[0], point_in_base_frame[1], point_in_base_frame[2]]),
        from_frame='franka_tool', 
        to_frame='world'
    )
        # No planning or execution, just return success
        return des_pose
    
    def get_pose_in_moveit_frame_from_end_effector_frame(self, pose: geometry_msgs.msg.Pose) -> geometry_msgs.msg.Pose:
        end_effector_in_base_frame = self.franka_moveit.get_transform()
        transform_mat =  np.array([[1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,1,-0.1034],
                                   [0,0,0,1]])
        pose_as_matrix = self.franka_moveit.pose_to_transformation_matrix(pose)
        #TODO: Check matmuls
        pose_as_matrix_in_moveit_frame = pose_as_matrix @ transform_mat @ end_effector_in_base_frame
        pose_in_moveit_frame = self.franka_moveit.transformation_matrix_to_pose(pose_as_matrix_in_moveit_frame)
        print(pose_in_moveit_frame)
        return pose_in_moveit_frame
    
    def goto_point(self, point: geometry_msgs.msg.Point) -> None:
        # Point is given in the base frame
        pose = geometry_msgs.msg.Pose() # Orientation initialized to 0,0,0,1 (quaternion)
        pose.position = point # Replace the position with the given point
        pose.orientation.w = 1.0
        # pose_moveit = self.get_pose_in_moveit_frame_from_end_effector_frame(pose)
        pose_moveit = pose
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "panda_end_effector"  # Set the frame to base frame
        pose_stamped.pose = pose
        # self.franka_moveit.add_box("hardcoded_point", pose_stamped, size=[0.01, 0.01, 0.01])
        plan = self.franka_moveit.get_plan_given_pose(pose_moveit) # Joint trajectory returned by the planner of shape N x 7
        # plan_smoothed = self.franka_moveit.get_straight_plan_given_pose(pose_moveit)
        self.franka_moveit.execute_plan(plan)
    
    def pick_at_point(self, point: geometry_msgs.msg.Point) -> None:
        # Goto point
        self.goto_point(point)        
        # Close gripper
        self.franka_moveit.fa.close_gripper()
        
    def place_at_point(self, point: geometry_msgs.msg.Point) -> None:
        # Goto point
        self.goto_point(point)
        # Open gripper
        self.franka_moveit.fa.open_gripper()
    
    def add_collision_boxes(self, collision_boxes) -> None:
        # collision_boxes = list(str, Pose, list)
        for name, pose, dimensions in collision_boxes:
            self.franka_moveit.add_box(name, pose, dimensions)
        
    def get_hardcoded_point(self) -> geometry_msgs.msg.Point:
        # Hardcoded point in the end effector frame
        x = 0
        y = 0
        z = 0.10
        # pose = geometry_msgs.msg.PoseStamped()
        # pose.header.frame_id = "panda_end_effector"
        # pose.pose.position.x = x
        # pose.pose.position.y = y
        # pose.pose.position.z = z
        # pose.pose.orientation.x = 1.0  # Default orientation
        # self.franka_moveit.add_box("hardcoded_point", pose, size=[0.01, 0.01, 0.01])  # Add a small box for visualization
        return geometry_msgs.msg.Point(x, y, z)
    
    def get_value_for_orange_centroid(self):
        """
        Retrieve the 3D coordinates of the orange object centroid.
        
        Returns:
            geometry_msgs.msg.Point: The 3D point coordinates of the orange object,
                                    or None if no data is received within timeout.
        """

        centroid_data = {'point': None}
        def callback(msg):
            centroid_data['point'] = msg.point
        
        # Subscribe to the orange centroid topic
        orange_sub = rospy.Subscriber('/color_tracker/orange_centroid_3d', 
                                    PointStamped, callback)
        
        # Wait for data with timeout
        timeout = rospy.Duration(2.0)  # 2 second timeout
        start_time = rospy.Time.now()
        
        while centroid_data['point'] is None:
            # Check if timeout has occurred
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Timeout waiting for orange centroid data")
                return None
            
            # Sleep a bit to avoid hogging CPU
            rospy.sleep(0.1)
        
        # Unsubscribe to avoid unnecessary callback executions
        orange_sub.unregister()
        
        # Return the point
        return centroid_data['point']

    
if __name__ == "__main__":
    

    # # Print Current Robot State (Joint Values and End Effector Pose)
    # sort = Sort_MoveIt()
    # sort.franka_moveit.print_robot_state()
    
    # # Add collision boxes
    # sort.franka_moveit.add_collision_boxes()
    # sort.move_to_second_home()
    # time.sleep(1)
    # x = sort.franka_moveit.get_transform()
    # point_ee = np.array([0, 0, 0.1-0.1034, 1]).T
    # point_base = x@point_ee
    # print(point_base[:3])
    
    # pose_in_ee =  geometry_msgs.msg.Pose()
    # pose_in_ee.orientation.x = 1.0
    
    # shelf_env = Shelf()
    # sort = Sort_MoveIt(shelf_env)
    # sort.franka_moveit.reset_joints()
    # sort.franka_moveit.print_robot_state()
       
    # sort.move_to_second_home()
    # time.sleep(1)
    
    # # breakpoint()
    # orange_centroid = sort.get_value_for_orange_centroid()
    # print(f"Orange Centroid: {orange_centroid}")
    
    # sort.franka_moveit.goto_pose(orange_centroid)    
    
    # # point_in_ee = sort.get_hardcoded_point()
    # # # print(point_in_ee)
    # print(orange_centroid)
    # sort.pick_at_point(orange_centroid)
    # # sort.pick_at_point(point_in_ee)
    
    
    
    # sort.move_to_second_home()
    
    # sort.franka_moveit.fa.open_gripper()
    
    
    #KEERTHI
    shelf_env = Shelf()
    sort = Sort_MoveIt(shelf_env)
    # sort.franka_moveit.get_pose()
    # sort.franka_moveit.reset_joints()
    # sort.franka_moveit.print_robot_state()
       
    # sort.move_to_second_home()
    # time.sleep(1)
    
    # breakpoint()
    # orange_centroid = sort.get_value_for_orange_centroid()
    # print(f"Orange Centroid: {orange_centroid}")
    
    des_pose = sort.goto_pose(orange_centroid) 
    # print(f"Desired Pose: {des_pose}")   
    sort.franka_moveit.goto_pose()
    
    

#catkin_make && source devel/setup.bash && rosrun sornt moveit_test.py 



    
    