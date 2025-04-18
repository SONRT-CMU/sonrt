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
        # joint_goal = [0.389478, 0.33705704, 0.34117744, -1.90028557, -1.17125388, 0.96495198, 0.05239409]
        joint_goal = [0.38918281, 0.26936133, 0.34682914, -1.89254621, -1.17081157, 0.96358994, -0.03604931]
        self.franka_moveit.goto_joint(joint_goal)

    def move_to_third_home(self):
        # joint_goal = [0.39537147, 0.88145527, 0.27661472, -1.74497851, -1.17776832, 1.1262399, 0.42056621]
        joint_goal =[ 0.39959001,  0.78209272,  0.29910651, -1.80425341, -1.15059544,  1.08383062, 0.41051894]

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
    
    
    def get_red_centroid(self):
        """
        Retrieve the 3D coordinates of the orange object centroid.
        
        Returns:
            geometry_msgs.msg.Point: The 3D point coordinates of the orange object,
                                    or None if no data is received within timeout.
        """

        centroid_data = {'point': None}
        def callback(msg):
            centroid_data['point'] = msg.point
        
        # Subscribe to the red centroid topic
        red_sub = rospy.Subscriber('/color_tracker/red_centroid_3d', 
                                    PointStamped, callback)
        
        # Wait for data with timeout
        timeout = rospy.Duration(2.0)  # 2 second timeout
        start_time = rospy.Time.now()
        
        while centroid_data['point'] is None:
            # Check if timeout has occurred
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Timeout waiting for red centroid data")
                return None
            
            # Sleep a bit to avoid hogging CPU
            rospy.sleep(0.1)
        
        # Unsubscribe to avoid unnecessary callback executions
        red_sub.unregister()
        
        # Return the point
        return centroid_data['point']
    
    def get_blue_centroid(self):
        """
        Retrieve the 3D coordinates of the orange object centroid.
        
        Returns:
            geometry_msgs.msg.Point: The 3D point coordinates of the orange object,
                                    or None if no data is received within timeout.
        """

        centroid_data = {'point': None}
        def callback(msg):
            centroid_data['point'] = msg.point
        
        # Subscribe to the blue centroid topic
        blue_sub = rospy.Subscriber('/color_tracker/blue_centroid_3d', 
                                    PointStamped, callback)
        
        # Wait for data with timeout
        timeout = rospy.Duration(2.0)  # 2 second timeout
        start_time = rospy.Time.now()
        
        while centroid_data['point'] is None:
            # Check if timeout has occurred
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Timeout waiting for blue centroid data")
                return None
            
            # Sleep a bit to avoid hogging CPU
            rospy.sleep(0.1)
        
        # Unsubscribe to avoid unnecessary callback executions
        blue_sub.unregister()
        
        # Return the point
        return centroid_data['point']
    
    def get_green_centroid(self):
        """
        Retrieve the 3D coordinates of the orange object centroid.
        
        Returns:
            geometry_msgs.msg.Point: The 3D point coordinates of the orange object,
                                    or None if no data is received within timeout.
        """

        centroid_data = {'point': None}
        def callback(msg):
            centroid_data['point'] = msg.point
        
        # Subscribe to the green centroid topic
        green_sub = rospy.Subscriber('/color_tracker/green_centroid_3d', 
                                    PointStamped, callback)
        
        # Wait for data with timeout
        timeout = rospy.Duration(2.0)  # 2 second timeout
        start_time = rospy.Time.now()
        
        while centroid_data['point'] is None:
            # Check if timeout has occurred
            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("Timeout waiting for green centroid data")
                return None
            
            # Sleep a bit to avoid hogging CPU
            rospy.sleep(0.1)
        
        # Unsubscribe to avoid unnecessary callback executions
        green_sub.unregister()
        
        # Return the point
        return centroid_data['point']    
    
    def pickup(self,color):
        # Get the orange centroid
        if color == 'red':
            centroid = self.get_red_centroid()
        elif color == 'blue':
            centroid = self.get_blue_centroid()
        elif color == 'green':
            centroid = self.get_green_centroid()
        
        if centroid is None:
            rospy.logerr("Failed to get orange centroid")
            return 
        
        T, T_ee_camera = sort.franka_moveit.get_transform_tf2()
        

        p_ee = np.array([centroid.x, centroid.y, centroid.z, 1]).T
        pickup_point = T@p_ee
        pickup_position = pickup_point[:3] / pickup_point[3]
        pickup_position[0] -= 0.01
        # pickup_position[2] += 0.01
        pickup_position[1] -= 0.03
        
        # Pick at the point
        current_pose = self.franka_moveit.fa.get_pose()
        pickup_pose = RigidTransform(
        rotation=current_pose.rotation,
        translation=pickup_position,
        from_frame='franka_tool',
        to_frame='world')

        self.pick_at_point(pickup_pose)

        time.sleep(0.5)
        # Move the robot 5 cm above the pickup position
        pickup_position[2] += 0.03  # Add 5 cm to the z-coordinate
        above_pickup_pose = RigidTransform(
            rotation=current_pose.rotation,
            translation=pickup_position,
            from_frame='franka_tool',
            to_frame='world')

        self.franka_moveit.fa.goto_pose(above_pickup_pose)

    def place(self,position,rotation):
        
        position[2] += 0.02
        pose = RigidTransform(
        rotation=rotation,
        translation=position,
        from_frame='franka_tool',
        to_frame='world')

        # Place the object at the given pose
        self.franka_moveit.fa.goto_pose(pose)
        time.sleep(0.5)
        self.franka_moveit.fa.open_gripper()


    
if __name__ == "__main__":
       

    position_1 = [0.61502854, -0.31966704, 0.38818833]
    rotation_1 = [
        [-0.01092505,  0.99991541, -0.00552877],
        [-0.06773011, -0.00625634, -0.99768402],
        [-0.99763421, -0.01052529,  0.06779403]
    ]

    # Position 2
    position_2 = [0.4496459, -0.32577249, 0.37486133]
    rotation_2 = [
        [-0.0159875, 0.99918275, -0.03686478],
        [-0.04397288, -0.0375361, -0.99832728],
        [-0.99889516, -0.01433971, 0.04453791]
    ]

    # Position 3
    position_3 = [0.30685658, -0.31731813, 0.38489819]
    rotation_3 = [
        [-0.08437571, 0.99554888, -0.04176058],
        [ 0.01714317, -0.04045304, -0.99903435],
        [-0.99627687, -0.08501014, -0.01365388]
    ]

    # Position 4
    position_4 = [0.62007213, -0.29471126, 0.09797535]
    rotation_4 = [
        [-0.06024339, 0.99798348, 0.01950539],
        [-0.06227433, 0.01574487, -0.99793483],
        [-0.99622959, -0.06133366, 0.0612014]
    ]

    # Position 5
    position_5 = [0.46596075, -0.29291366, 0.0993864]
    rotation_5 = [
        [-0.01725819,  0.99810282, -0.05893836],
        [ 0.03908043, -0.05822814, -0.99753802],
        [-0.99907739, -0.01951904, -0.03800211]
    ]

    # Position 6
    position_6 = [0.31284843, -0.29916375, 0.08518446]
    rotation_6 = [
        [-0.0108778, 0.99990185, -0.00766263],
        [ 0.06284333, -0.00696424, -0.99799907],
        [-0.99795448, -0.01133758, -0.06276261]
    ]
    
    ### FSM
    
    shelf_env = Shelf()

    sort = Sort_MoveIt(shelf_env)
       
    #pickup_red
    sort.move_to_second_home()
    sort.pickup(color='red')
    time.sleep(0.5)
    sort.move_to_second_home()

    sort.place(position=position_1, rotation=rotation_1)
    
    sort.move_to_second_home()
    
    
    
    #pickup_blue
    sort.move_to_third_home()
    sort.pickup(color='blue')
    time.sleep(0.5)
    sort.move_to_third_home()
    sort.move_to_second_home()
    sort.place(position=position_3, rotation=rotation_3)
    # Use the pose with FrankaPy
    # sort.franka_moveit.fa.goto_pose(pose)
    sort.franka_moveit.fa.open_gripper()
    
    sort.move_to_second_home()
    
    
    # sort.move_to_third_home()
        
    
    
    
    
    

    

#catkin_make && source devel/setup.bash && rosrun sornt moveit_test.py 



    
    