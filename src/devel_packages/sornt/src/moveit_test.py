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
        
        # joint_goal = [0.09446621,  0.23436419,  0.51143809, -2.13887694, -1.22993535,  1.01465016, 0.07993947]
        # joint_goal = [0.36640268, 0.38592696, 0.49458685, -1.81152975, -1.12258912, 0.85101375, 0.04232636]
        joint_goal = [0.389478, 0.33705704, 0.34117744, -1.90028557, -1.17125388, 0.96495198, 0.05239409]

        self.franka_moveit.goto_joint(joint_goal)

    def move_to_third_home(self):
        joint_goal = [0.39537147, 0.88145527, 0.27661472, -1.74497851, -1.17776832, 1.1262399, 0.42056621]
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


    def pick_at_point(self, point: geometry_msgs.msg.Point) -> None:
        # Goto point
        self.franka_moveit.fa.goto_pose(point)        
        # Close gripper
        time.sleep(0.5)
        self.franka_moveit.fa.close_gripper()
        
    def place_at_point(self, point: geometry_msgs.msg.Point) -> None:
        # Goto point
        self.franka_moveit.fa.goto_pose(point)
        time.sleep(0.5)
        # Open gripper
        self.franka_moveit.fa.open_gripper()
    
    def add_collision_boxes(self, collision_boxes) -> None:
        # collision_boxes = list(str, Pose, list)
        for name, pose, dimensions in collision_boxes:
            self.franka_moveit.add_box(name, pose, dimensions)
    
    
    def get_orange_centroid(self):
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
    
    def pickup(self):
        # Get the orange centroid
        orange_centroid = self.get_orange_centroid()
        
        if orange_centroid is None:
            rospy.logerr("Failed to get orange centroid")
            return 
        
        T, T_ee_camera = sort.franka_moveit.get_transform_tf2()

        p_ee = np.array([orange_centroid.x, orange_centroid.y, orange_centroid.z, 1]).T
        pickup_point = T@p_ee
        pickup_position = pickup_point[:3] / pickup_point[3]
        pickup_position[0] -= 0.02
        pickup_position[2] += 0.02
        pickup_position[1] -= 0.04
        
        # Pick at the point
        current_pose = self.franka_moveit.fa.get_pose()
        pickup_pose = RigidTransform(
        rotation=current_pose.rotation,
        translation=pickup_position,
        from_frame='franka_tool',
        to_frame='world')

        # sort.franka_moveit.fa.goto_pose(pickup_pose)
        self.pick_at_point(pickup_pose)

        time.sleep(0.5)
        # Move the robot 5 cm above the pickup position
        pickup_position[2] += 0.05  # Add 5 cm to the z-coordinate
        above_pickup_pose = RigidTransform(
            rotation=current_pose.rotation,
            translation=pickup_position,
            from_frame='franka_tool',
            to_frame='world')

        self.franka_moveit.fa.goto_pose(above_pickup_pose)




    
if __name__ == "__main__":
    

    
    shelf_env = Shelf()

    sort = Sort_MoveIt(shelf_env)
    # sort.franka_moveit.print_robot_state()
       
    sort.move_to_second_home()
    sort.pickup()
    time.sleep(0.5)
    sort.move_to_second_home()
    sort.move_to_third_home()
    sort.franka_moveit.fa.open_gripper()


    # NOTE: Position 5 (Bottom shelf middle)
    # Enter a number: 1
    # pose: 
    # Tra: [ 0.46596075 -0.29291366  0.0993864 ]
    # Rot: [[-0.01725819  0.99810282 -0.05893836]
    # [ 0.03908043 -0.05822814 -0.99753802]
    # [-0.99907739 -0.01951904 -0.03800211]]


    position = [0.46596075, -0.29291366, 0.0993864]
    rotation = [
        [-0.01725819,  0.99810282, -0.05893836],
        [ 0.03908043, -0.05822814, -0.99753802],
        [-0.99907739, -0.01951904, -0.03800211]
    ]

    pose = RigidTransform(
        rotation=np.array(rotation),
        translation=np.array(position),
        from_frame='franka_tool',
        to_frame='world'
    )

    # Use the pose with FrankaPy
    # sort.franka_moveit.fa.goto_pose(pose)
        
    
    
    
    
    

    

#catkin_make && source devel/setup.bash && rosrun sornt moveit_test.py 



    
    