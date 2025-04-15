# Author: Vibhakar Mohta vib2810@gmail.com
#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import numpy as np
from moveit_msgs.msg import PlanningScene
import scipy.spatial.transform as spt
import tf
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft
from sensor_msgs.msg import CameraInfo

sys.path.append("/home/ros_ws/src/git_packages/frankapy")
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
import copy

import time
from autolab_core import RigidTransform

class MoveItPlanner():
    def __init__(self) -> None: #None means no return value
        print("Initializing MoveIt Planner")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        print("MoveIt Node Initialized")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        

        group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        # NOTE: Changed planning reference frame
        self.group.set_pose_reference_frame("panda_end_effector")
        self.group.set_planner_id("RRTConnect")

        print("MoveIt Group Initialized")
        self.group.set_end_effector_link("panda_hand")
        self.obs_pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        
        # used to visualize the planned path
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        print("variables set")
        planning_frame = self.group.get_planning_frame()
        eef_link = self.group.get_end_effector_link()

        print("---------Moveit Planner Class Initialized---------")
        print("Planning frame: ", planning_frame)
        print("End effector: ", eef_link)
        print("Robot Groups:", self.robot.get_group_names())

        # frankapy 
        self.pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        self.fa = FrankaArm(init_node = False)
    
    # Utility Functions
    def print_robot_state(self):
        print("Joint Values:\n", self.group.get_current_joint_values())
        print("Pose Values (panda_hand):\n", self.group.get_current_pose())
        print("Pose Values (panda_end_effector):\nNOTE: Quaternion is (w,x,y,z)\n", self.fa.get_pose())
     
    def reset_joints(self):
        self.fa.reset_joints()

    def goto_joint(self, joint_goal):
        # self.fa.goto_joints(joint_goal, duration=5, dynamic=True, buffer_time=10)
        
        #KEERTHI        
        self.fa.goto_joints(joint_goal, use_impedance=False)

    def goto_pose(self,pose_goal):

        self.fa.goto_pose(pose_goal,duration=5,dynamic=True,buffer_time=10)
           
    #KEERTHI
    def goto_pose_test(self):
        self.fa.open_gripper()
        # self.fa.reset_joints()
        initial_joint = self.fa.get_joints()
        print(f'Initial joint: {initial_joint}')
        
        home_joint = [0.09446621,  0.23436419,  0.51143809, -2.13887694, -1.22993535,  1.01465016, 0.07993947]
        self.goto_joint(home_joint)
        print(f'Home joint: {home_joint}')
        
        self.fa.reset_joints()
        des_pose = RigidTransform(rotation=np.array([
            [-0.02110541, 0.99952769, -0.02190237],
            [-0.07263959, -0.02338222, -0.99708407],
            [-0.99712527, -0.01945289, 0.07310018]]),
        # translation=[0.45209972, 0.04327647, 0.36306605],
        translation=[0.54990976, -0.38635197, 0.40136056],
        # translation=[0.6520895, 0.24325199, 0.16303764],
        from_frame='franka_tool', 
        to_frame='world'
    )
        self.fa.goto_pose(des_pose, use_impedance=False)
        des_joint = self.fa.get_joints()
        print(f'Desired joint: {des_joint}')
        
        self.goto_joint(initial_joint)
        self.goto_joint(home_joint)
        self.goto_joint(des_joint)
        self.fa.close_gripper()
        self.goto_joint(home_joint)
        self.goto_joint(des_joint)
        self.fa.open_gripper() 
        self.goto_joint(home_joint)
        self.goto_joint(initial_joint)
        
        
    #KEERTHI
    def get_pose(self):
        des_pose = self.fa.get_pose()
        print(f'Current pose: {des_pose}')
        return des_pose
        
    
    def get_plan_given_pose(self, pose_goal: geometry_msgs.msg.Pose):
        """
        Plans a trajectory given a tool pose goal
        Returns joint_values 
        joint_values: numpy array of shape (N x 7)
        """
        output = self.group.plan(pose_goal)
        plan = output[1]
        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values
    
    def get_plan_given_joint(self, joint_goal_list):
        """
        Plans a trajectory given a joint goal
        Returns joint_values and moveit plan
        joint_values: numpy array of shape (N x 7)
        plan: moveit_msgs.msg.RobotTrajectory object
        """
        joint_goal = sensor_msgs.msg.JointState()
        joint_goal.name = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        joint_goal.position = joint_goal_list

        output = self.group.plan(joint_goal)
        plan = output[1]
        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values, plan

    def get_straight_plan_given_pose(self, pose_goal: geometry_msgs.msg.Pose):
        """
        pose_goal: geometry_msgs.msg.Pose
        Plans a trajectory given a tool pose goal
        Returns joint_values
        joint_values: numpy array of shape (N x 7)
        """
        waypoints = []
        waypoints.append(copy.deepcopy(pose_goal))

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, 0.01 # waypoints to follow  # eef_step
        )  # jump_threshold

        joint_values = []
        for i in range(len(plan.joint_trajectory.points)):
            joint_values.append(plan.joint_trajectory.points[i].positions)
        joint_values = np.array(joint_values)
        return joint_values

    def execute_plan(self, joints_traj):
        """
        joints_traj shape: (N x 7)
        """
        # interpolate the trajectory
        num_interp_slow = 50 # number of points to interpolate for the start and end of the trajectory
        num_interp = 20 # number of points to interpolate for the middle part of the trajectory
        interpolated_traj = []
        t_linear = np.linspace(1/num_interp, 1, num_interp)
        t_slow = np.linspace(1/num_interp_slow, 1, num_interp_slow)
        t_ramp_up = t_slow**2
        t_ramp_down = 1 - (1-t_slow)**2

        interpolated_traj.append(joints_traj[0,:])
        for t_i in range(len(t_ramp_up)):
            dt = t_ramp_up[t_i]
            interp_traj_i = joints_traj[1,:]*dt + joints_traj[0,:]*(1-dt)
            interpolated_traj.append(interp_traj_i)
            
        for i in range(2, joints_traj.shape[0]-1):
            for t_i in range(len(t_linear)):
                dt = t_linear[t_i]
                interp_traj_i = joints_traj[i,:]*dt + joints_traj[i-1,:]*(1-dt)
                interpolated_traj.append(interp_traj_i)

        for t_i in range(len(t_ramp_down)):
            dt = t_ramp_down[t_i]
            interp_traj_i = joints_traj[-1,:]*dt + joints_traj[-2,:]*(1-dt)
            interpolated_traj.append(interp_traj_i)

        interpolated_traj = np.array(interpolated_traj)
        print("Interpolated Trajectory: ", interpolated_traj)

        print('Executing joints trajectory of shape: ', interpolated_traj.shape)

        rate = rospy.Rate(50)
        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        self.fa.goto_joints(interpolated_traj[1], duration=5, dynamic=True, buffer_time=10)
        init_time = rospy.Time.now().to_time()
        for i in range(2, interpolated_traj.shape[0]):
            traj_gen_proto_msg = JointPositionSensorMessage(
                id=i, timestamp=rospy.Time.now().to_time() - init_time, 
                joints=interpolated_traj[i]
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
            )
            self.pub.publish(ros_msg)
            rate.sleep()

        # Stop the skill
        # Alternatively can call fa.stop_skill()
        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
            termination_handler_sensor_msg=sensor_proto2ros_msg(
                term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
            )
        self.pub.publish(ros_msg)
    
    # Test Functions
    def unit_test_joint(self, execute = False, guided = False):
        """
        Unit test for joint trajectory planning
        Resets to home and plans to the joint goal
        Displays the planned path to a fixed joint goal on rviz

        Parameters
        ----------
        execute: bool
            If True, executes the planned trajectory
        guided: bool
            If True, runs guide mode where the user can move the robot to a desired joint goal
            Else, uses a fixed joint goal
        """
        if guided:
            print("Running Guide Mode, Move Robot to Desired Pose")
            self.fa.run_guide_mode(10, block=True)
            self.fa.stop_skill()
            joint_goal_list = self.fa.get_joints()
            print("Joint Goal: ", joint_goal_list)
        else:
            # A random joint goal
            joint_goal_list = [6.14813255e-02 ,4.11382927e-01, 6.80023936e-02,-2.09547337e+00,-2.06094866e-03,2.56799173e+00, 9.20088362e-01]
        print("Resetting Joints")
        self.fa.reset_joints()
        plan_joint_vals, plan_joint = self.get_plan_given_joint(joint_goal_list)
        print("Planned Path Shape: ", plan_joint_vals.shape)
        if execute: 
            print("Executing Plan")
            self.execute_plan(plan_joint_vals)
    
    def unit_test_pose(self, execute = False, guided = False):
        """
        Unit test for pose trajectory planning
        Resets to home and plans to the pose goal
        Displays the planned path to a fixed pose goal on rviz

        Parameters
        ----------
        execute: bool
            If True, executes the planned trajectory
        guided: bool
            If True, runs guide mode where the user can move the robot to a desired pose goal
            Else, uses a fixed pose goal
        """
        print("Unit Test for Tool Pose Trajectory Planning")
        if guided:
            print("Running Guide Mode, Move Robot to Desired Pose")
            self.fa.run_guide_mode(10, block=True)
            self.fa.stop_skill()
            pose_goal_fa = self.fa.get_pose()
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = pose_goal_fa.translation[0]
            pose_goal.position.y = pose_goal_fa.translation[1]
            pose_goal.position.z = pose_goal_fa.translation[2]
            pose_goal.orientation.w = pose_goal_fa.quaternion[0]
            pose_goal.orientation.x = pose_goal_fa.quaternion[1]
            pose_goal.orientation.y = pose_goal_fa.quaternion[2]
            pose_goal.orientation.z = pose_goal_fa.quaternion[3]
        else:
            pose_goal = geometry_msgs.msg.Pose()
            # a random test pose
            pose_goal.position.x = 0.5
            pose_goal.position.y = 0.2
            pose_goal.position.z = -0.5
            pose_goal.orientation.x = 1.0
            pose_goal.orientation.y = 0.0
            pose_goal.orientation.z = 0.0
            pose_goal.orientation.w = 0.0
            # pose_goal.orientation.x = 0.0
            # pose_goal.orientation.y = 0.0
            # pose_goal.orientation.z = 0.0 
            # pose_goal.orientation.w = -1.0
        
        # Convert to moveit pose
        pose_goal = self.get_moveit_pose_given_frankapy_pose(pose_goal)
        print("Pose Goal: ", pose_goal)
        print("Resetting Joints")
        self.fa.reset_joints()
        plan_pose = self.get_plan_given_pose(pose_goal)
        print("Planned Path Shape: ", plan_pose.shape)
        if execute:
            print("Executing Plan")
            self.execute_plan(plan_pose)

    def get_moveit_pose_given_frankapy_pose(self, pose):
        """
        Converts a frankapy pose (in panda_end_effector frame) to a moveit pose (in panda_hand frame) 
        by adding a 10 cm offset to z direction
        """
        transform_mat =  np.array([[1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,1,-0.1034],
                                   [0,0,0,1]])
        pose_mat = self.pose_to_transformation_matrix(pose)
        transformed = pose_mat @ transform_mat
        pose_goal = self.transformation_matrix_to_pose(transformed)
        return pose_goal
    
    def pose_to_transformation_matrix(self, pose):
        """
        Converts geometry_msgs/Pose to a 4x4 transformation matrix
        """
        T = np.eye(4)
        T[0,3] = pose.position.x
        T[1,3] = pose.position.y
        T[2,3] = pose.position.z
        r = spt.Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T[0:3, 0:3] = r.as_matrix()
        return T

    def transformation_matrix_to_pose(self, trans_mat):   
        """
        Converts a 4x4 transformation matrix to geometry_msgs/Pose
        """
        out_pose = geometry_msgs.msg.Pose()
        out_pose.position.x = trans_mat[0,3]
        out_pose.position.y = trans_mat[1,3]
        out_pose.position.z = trans_mat[2,3]

        #convert rotation matrix to quaternion
        r = spt.Rotation.from_matrix(trans_mat[0:3, 0:3])
        quat = r.as_quat() 
        out_pose.orientation.x = quat[0]
        out_pose.orientation.y = quat[1]
        out_pose.orientation.z = quat[2]
        out_pose.orientation.w = quat[3] 
        return out_pose

    def add_box(self, name, pose: geometry_msgs.msg.PoseStamped(), size):
        """
        Adds a collision box to the planning scene

        Parameters
        ----------
        name : str
            Name of the box
        pose : geometry_msgs.msg.PoseStamped
            Pose of the box (Centroid and Orientation)
        size : list
            Size of the box in x, y, z  
        """
        self.scene.add_box(name, pose, size)

    def remove_box(self, name):
        self.scene.remove_world_object(name)

    def add_mesh(self, name, pose: geometry_msgs.msg.PoseStamped(),filename,size):
        """
        Adds a collision box to the planning scene

        Parameters
        ----------
        name : str
            Name of the box
        pose : geometry_msgs.msg.PoseStamped
            Pose of the box (Centroid and Orientation)
        size : list
            Size of the box in x, y, z  
        """
        self.scene.add_mesh(name, pose, filename, size=[1,1,1])

    def remove_box(self, name):
        self.scene.remove_world_object(name)     
    
    def get_transform(self,):

        listener = tf.TransformListener()

        # Wait for the transforms to be available
        listener.waitForTransform("panda_link0", "panda_end_effector", rospy.Time(0), rospy.Duration(4.0))
        (trans_link0_to_ee, rot_link0_to_ee) = listener.lookupTransform("panda_link0", "panda_end_effector", rospy.Time(0))
        # Convert the rotation quaternion to a 4x4 transformation matrix
        transform_link0_to_ee = tf.transformations.compose_matrix(translate=trans_link0_to_ee, angles=tf.transformations.euler_from_quaternion(rot_link0_to_ee))
        
        return transform_link0_to_ee

    def transform_to_matrix(self,transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        rotation_matrix = tft.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        T_matrix = np.eye(4)
        T_matrix[:3, :3] = rotation_matrix[:3, :3]
        T_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        
        return T_matrix

    def get_transform_tf2(self,):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1)
        try:
            transform = tf_buffer.lookup_transform('panda_link0', 'panda_end_effector', rospy.Time(0))

            T_link0_ee = self.transform_to_matrix(transform)
            T_ee_camera = np.eye(4)
            R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
            T_ee_camera[:3, :3] = R
            T_ee_camera[:3, 3] = np.array([-0.04, 0, -0.06])
            T = T_link0_ee @ T_ee_camera
            return T, T_ee_camera

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            #rospy.logerr("Transform error: %s", e)
            return None
    
    def add_collision_boxes(self):
        base_pose = geometry_msgs.msg.PoseStamped()
        base_pose.header.frame_id = "panda_link0"
        base_pose.pose.position.x = 0.27+(0.55/2)
        base_pose.pose.position.y = 0.0
        base_pose.pose.position.z = 0.01/2
        base_pose.pose.orientation.x = 1.0
        base_pose.pose.orientation.y = 0.0
        base_pose.pose.orientation.z = 0.0
        base_pose.pose.orientation.w = 0.0
        self.add_box("base", base_pose, [0.55, 1.08, 0.01])
        # franka_moveit.add_mesh("shelf",box_pose,"./shelf.stl",[0.2, 0.2, 0.2])

        shelf_1 = geometry_msgs.msg.PoseStamped()
        shelf_1.header.frame_id = "panda_link0"
        shelf_1.pose.position.x = 0.22 + (0.6/2)
        shelf_1.pose.position.y = -0.245-0.145
        shelf_1.pose.position.z = 0.02
        shelf_1.pose.orientation.x = 1.0
        shelf_1.pose.orientation.y = 0.0
        shelf_1.pose.orientation.z = 0.0
        shelf_1.pose.orientation.w = 0.0
        self.add_box("shelf_1", shelf_1, [0.6, 0.29, 0.01])

        shelf_2 = geometry_msgs.msg.PoseStamped()
        shelf_2.header.frame_id = "panda_link0"
        shelf_2.pose.position.x = 0.22 + (0.6/2)
        shelf_2.pose.position.y = -0.245-0.145
        shelf_2.pose.position.z = 0.02 + 0.28
        shelf_2.pose.orientation.x = 1.0
        shelf_2.pose.orientation.y = 0.0
        shelf_2.pose.orientation.z = 0.0
        shelf_2.pose.orientation.w = 0.0
        self.add_box("shelf_2", shelf_2, [0.6, 0.29, 0.01])

        shelf_3 = geometry_msgs.msg.PoseStamped()
        shelf_3.header.frame_id = "panda_link0"
        shelf_3.pose.position.x = 0.22 + (0.6/2)
        shelf_3.pose.position.y = -0.245-0.145
        shelf_3.pose.position.z = 0.02 + 0.28 + 0.28
        shelf_3.pose.orientation.x = 1.0
        shelf_3.pose.orientation.y = 0.0
        shelf_3.pose.orientation.z = 0.0
        shelf_3.pose.orientation.w = 0.0
        self.add_box("shelf_3", shelf_3, [0.6, 0.29, 0.01])

        shelf_4 = geometry_msgs.msg.PoseStamped()
        shelf_4.header.frame_id = "panda_link0"
        shelf_4.pose.position.x = 0.82
        shelf_4.pose.position.y = -0.245-0.145
        shelf_4.pose.position.z = 0.02 + 0.3
        shelf_4.pose.orientation.x = 1.0
        shelf_4.pose.orientation.y = 0.0
        shelf_4.pose.orientation.z = 0.0
        shelf_4.pose.orientation.w = 0.0
        self.add_box("shelf_4", shelf_4, [0.01, 0.29, 0.6])

        shelf_5 = geometry_msgs.msg.PoseStamped()
        shelf_5.header.frame_id = "panda_link0"
        shelf_5.pose.position.x = 0.22
        shelf_5.pose.position.y = -0.245-0.145
        shelf_5.pose.position.z = 0.02 + 0.3
        shelf_5.pose.orientation.x = 1.0
        shelf_5.pose.orientation.y = 0.0
        shelf_5.pose.orientation.z = 0.0
        shelf_5.pose.orientation.w = 0.0
        self.add_box("shelf_5", shelf_5, [0.01, 0.29, 0.6])

if __name__ == "__main__":
    franka_moveit = MoveItPlanner()

    # Print Current Robot State (Joint Values and End Effector Pose)
    franka_moveit.print_robot_state()


    


    # Adding and removing obstacle boxes to planning scene
    base_pose = geometry_msgs.msg.PoseStamped()
    base_pose.header.frame_id = "panda_link0"
    base_pose.pose.position.x = 0.27+(0.55/2)
    base_pose.pose.position.y = 0.0
    base_pose.pose.position.z = 0.01/2
    base_pose.pose.orientation.x = 1.0
    base_pose.pose.orientation.y = 0.0
    base_pose.pose.orientation.z = 0.0
    base_pose.pose.orientation.w = 0.0
    franka_moveit.add_box("base", base_pose, [0.55, 1.08, 0.01])
    # franka_moveit.add_mesh("shelf",box_pose,"./shelf.stl",[0.2, 0.2, 0.2])

    shelf_1 = geometry_msgs.msg.PoseStamped()
    shelf_1.header.frame_id = "panda_link0"
    shelf_1.pose.position.x = 0.22 + (0.6/2)
    shelf_1.pose.position.y = -0.245-0.145
    shelf_1.pose.position.z = 0.02
    shelf_1.pose.orientation.x = 1.0
    shelf_1.pose.orientation.y = 0.0
    shelf_1.pose.orientation.z = 0.0
    shelf_1.pose.orientation.w = 0.0
    franka_moveit.add_box("shelf_1", shelf_1, [0.6, 0.29, 0.01])

    shelf_2 = geometry_msgs.msg.PoseStamped()
    shelf_2.header.frame_id = "panda_link0"
    shelf_2.pose.position.x = 0.22 + (0.6/2)
    shelf_2.pose.position.y = -0.245-0.145
    shelf_2.pose.position.z = 0.02 + 0.28
    shelf_2.pose.orientation.x = 1.0
    shelf_2.pose.orientation.y = 0.0
    shelf_2.pose.orientation.z = 0.0
    shelf_2.pose.orientation.w = 0.0
    franka_moveit.add_box("shelf_2", shelf_2, [0.6, 0.29, 0.01])

    shelf_3 = geometry_msgs.msg.PoseStamped()
    shelf_3.header.frame_id = "panda_link0"
    shelf_3.pose.position.x = 0.22 + (0.6/2)
    shelf_3.pose.position.y = -0.245-0.145
    shelf_3.pose.position.z = 0.02 + 0.28 + 0.28
    shelf_3.pose.orientation.x = 1.0
    shelf_3.pose.orientation.y = 0.0
    shelf_3.pose.orientation.z = 0.0
    shelf_3.pose.orientation.w = 0.0
    franka_moveit.add_box("shelf_3", shelf_3, [0.6, 0.29, 0.01])

    shelf_4 = geometry_msgs.msg.PoseStamped()
    shelf_4.header.frame_id = "panda_link0"
    shelf_4.pose.position.x = 0.82
    shelf_4.pose.position.y = -0.245-0.145
    shelf_4.pose.position.z = 0.02 + 0.3
    shelf_4.pose.orientation.x = 1.0
    shelf_4.pose.orientation.y = 0.0
    shelf_4.pose.orientation.z = 0.0
    shelf_4.pose.orientation.w = 0.0
    franka_moveit.add_box("shelf_4", shelf_4, [0.01, 0.29, 0.6])

    shelf_5 = geometry_msgs.msg.PoseStamped()
    shelf_5.header.frame_id = "panda_link0"
    shelf_5.pose.position.x = 0.22
    shelf_5.pose.position.y = -0.245-0.145
    shelf_5.pose.position.z = 0.02 + 0.3
    shelf_5.pose.orientation.x = 1.0
    shelf_5.pose.orientation.y = 0.0
    shelf_5.pose.orientation.z = 0.0
    shelf_5.pose.orientation.w = 0.0
    franka_moveit.add_box("shelf_5", shelf_5, [0.01, 0.29, 0.6])
    
#     # franka_moveit.unit_test_pose(execute=True, guided=True)
#     # franka_moveit.unit_test_joint(execute=True, guided=True) 
    
# #     [ 0.09446621  0.23436419  0.51143809 -2.13887694 -1.22993535  1.01465016
# #   0.07993947]
# #   x: 0.5013639092449318
# #   y: -0.2494264032628718
# #   z: 0.3799419026484181
# # orientation: 
# #   x: 0.505567635930478
# #   y: 0.477905251563738
# #   z: -0.4746004949978282
# #   w: 0.5392237997114783

    second_home, _ = franka_moveit.get_plan_given_joint([0.09446621,  0.23436419,  0.51143809, -2.13887694, -1.22993535,  1.01465016, 0.07993947])
    
    franka_moveit.execute_plan(second_home)
    
    time.sleep(2)
    
    # inverted_transform = franka_moveit.get_transform()
    # inverted_translation = inverted_transform[:3, 3]
    
    
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.5013639092449318 + inverted_translation[0]
    # pose_goal.position.y = -0.2494264032628718 + inverted_translation[1]
    # pose_goal.position.z = 0.3799419026484181 + inverted_translation[2]
    # pose_goal.orientation.x = 0.505567635930478
    # pose_goal.orientation.y = 0.477905251563738
    # pose_goal.orientation.z = -0.4746004949978282
    # pose_goal.orientation.w = 0.5392237997114783
    
    # plan_pose= franka_moveit.get_plan_given_pose(pose_goal)
    # franka_moveit.execute_plan(plan_pose)

    
    # time.sleep(2)

    # pre_grasp,_ = franka_moveit.get_plan_given_joint([0.02590275, -0.05521367, -0.01400492, -2.41612441, -1.58389572,  1.56879717 ,0.0200526])
    # home_two,_ = franka_moveit.get_plan_given_joint([0.42227243,  0.67925061,  0.23711456, -1.90579146, -1.10585297,  1.1212435, 0.34713481])
    # franka_moveit.execute_plan(home_two)
    










    # time.sleep(2)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.4827270947256986
    # pose_goal.position.y = -0.23732611220252173
    # pose_goal.position.z = 0.09799460920877211
    # pose_goal.orientation.x = -0.4920283194781625
    # pose_goal.orientation.y = -0.48546365372105876
    # pose_goal.orientation.z = 0.5423960222098694
    # pose_goal.orientation.w = -0.4775350550883214
    
    # plan_pose= franka_moveit.get_plan_given_pose(pose_goal)
    # franka_moveit.execute_plan(plan_pose)





    # Convert to moveit pose
    # pose_goal = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)
    # print("Pose Goal: ", pose_goal)
    # print("Resetting Joints")
    # self.fa.reset_joints()

    # print(plan_pose)
    # plan_pose = franka_moveit.get_straight_plan_given_pose(pose_goal)
    
    # # print("Planned Path Shape: ", plan_pose.shape)
    # # if execute:
    # print("Executing Plan")

    # time.sleep(2)

    # t_ee_world = franka_moveit.fa.get_pose()
    # t_ee_world.translation += [0,-0.1,0]
    # franka_moveit.fa.goto_pose(t_ee_world)

    # franka_moveit.fa.close_gripper()


    # t_ee_world = franka_moveit.fa.get_pose()
    # t_ee_world.translation += [0,+0.2,0.05]

    # franka_moveit.fa.goto_pose(t_ee_world)

    # franka_moveit.fa.reset_joints()


    # time.sleep(10)

    

    # Remove added obstacle box
    # franka_moveit.remove_box("box")