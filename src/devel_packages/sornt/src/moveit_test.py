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

from src.devel_packages.sornt.src.utils.shelf import Shelf

# from utils.shelf import Shelf

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

    
    def convert_point_to_pose(self, point):
        pose = geometry_msgs.msg.Pose() # Orientation initialized to 0,0,0,1 (quaternion)
        pose.position = point # Replace the position with the given point
        pose.orientation.w = 1.0

        return pose

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
    

    
    shelf_env = Shelf()

    sort = Sort_MoveIt(shelf_env)
    sort.franka_moveit.print_robot_state()
       
    sort.move_to_second_home()

    orange_centroid = sort.get_value_for_orange_centroid()
    
    print(orange_centroid)
    
    T, T_ee_camera = sort.franka_moveit.get_transform_tf2()
    
    p_ee = np.array([orange_centroid.x, orange_centroid.y, orange_centroid.z, 1]).T
    
    print(T@p_ee)
    
    

    

#catkin_make && source devel/setup.bash && rosrun sornt moveit_test.py 



    
    