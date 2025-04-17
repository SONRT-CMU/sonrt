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
    
    #KEERTHI
    def get_camtobase_transform(self):
        ee_transform = self.franka_moveit.fa.get_pose()
        T_ee_base = ee_transform.matrix 
        print("T_ee_base:", T_ee_base)  
        T_camera_ee = np.eye(4)
        T_camera_ee[:3, 3] = np.array([0.03, -0.03, -0.01])
        # T_camera_ee[:3, 3] = np.array([-0.04742764, 0.10384115, 0.032812011])
        T_camera_to_base = T_ee_base @ T_camera_ee
        
        return T_camera_to_base

    
    #KEERTHI
    def pickandplace(self, point, T):
        print("point in camera:", point)
        
        
        
        # p_camera = np.array([orange_centroid.x, orange_centroid.y, orange_centroid.z, 1])
        # p_base = T @ p_camera
        # print("T_camera_to_base:", T)
        # print("point in base:", p_base)
            
        # des_pose = RigidTransform(
        #         rotation=np.array([
        #             [-0.02110541,  0.99952769, -0.02190237],
        #             [-0.07263959, -0.02338222, -0.99708407],
        #             [-0.99712527, -0.01945289, 0.07310018]
        #         ]),
        #         translation=p_base[:3].tolist(), 
        #         from_frame='franka_tool', 
        #         to_frame='world'
        #     )
        # sort.franka_moveit.reset_joints()
        # sort.franka_moveit.goto_pose(des_pose)    
        # sort.franka_moveit.fa.close_gripper()
        
        # current_pose = sort.franka_moveit.get_pose()
        # lift_translation = current_pose.translation.copy()
        # lift_translation[2] += 0.05
        # lift_pose = RigidTransform(
        #     rotation=current_pose.rotation,
        #     translation=lift_translation, 
        #     from_frame='franka_tool',
        #     to_frame='world'
        # )
        # sort.franka_moveit.goto_pose(lift_pose)
        
        # sort.move_to_second_home()
        # sort.franka_moveit.goto_joint(position3)
        # sort.franka_moveit.fa.open_gripper()
        
        # current_pose = sort.franka_moveit.get_pose()
        # preplace_translation = current_pose.translation.copy()
        # preplace_translation[1] += 0.10
        # preplace_pose = RigidTransform(
        #     rotation=current_pose.rotation,
        #     translation=preplace_translation, 
        #     from_frame='franka_tool',
        #     to_frame='world'
        # )        
        # sort.franka_moveit.goto_pose(preplace_pose)
        # sort.move_to_second_home()

    
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
    sort.franka_moveit.reset_joints()
    sort.franka_moveit.print_robot_state()
    
    position1 = [ 0.04932081,  0.41680444, -0.28264505, -1.80764826, -1.65895955,  1.76036052, -0.13009759]
    position2 = [ 0.56087855,  0.0201718,  -0.83533652, -2.35995647, -1.76507356,  1.71175639,  0.02144443]
    position3 = [ 0.0267715,  -0.48488081, -0.38927448, -2.78192472, -1.88040301,  1.66381664, -0.12541168]
    position4 = [ 0.15417526,  1.00905677, -0.36440428, -1.62995756, -1.50209554,  1.81501208,  0.19394939]
    position5 = [ 0.63118503,  0.94457377, -0.83804126, -2.16908984, -1.25600768,  2.19826817,  0.40085538]
    position6 = [ 0.09193547,  0.55592332, -0.35134907, -2.57623927, -1.769613,    1.81996532,  0.77395573]
       
    # sort.move_to_second_home()

    # des_pose = RigidTransform(
    #     rotation=np.array([
    #         [-0.01725819,  0.99810282, -0.05893836],
    #         [ 0.03908043, -0.05822814, -0.99753802],
    #         [-0.99907739, -0.01951904, -0.03800211]
    #     ]),
    #     translation=[ 0.46596075, -0.529291366,  -0.04 ], 
    #     from_frame='franka_tool', 
    #     to_frame='world'
    #     )
    # sort.franka_moveit.reset_joints()
    # sort.franka_moveit.goto_pose(des_pose)
    
    # for i in range(3):            
    #     print("i:", i)
    #     orange_centroid = sort.get_value_for_orange_centroid()  
    #     T = sort.get_camtobase_transform()        
    #     sort.pickandplace(orange_centroid, T)
    #     time.sleep(5)


#catkin_make && source devel/setup.bash && rosrun sornt moveit_test.py 



    
    