from geometry_msgs.msg import PoseStamped

class Shelf():
    def __init__(self):
        # Adding and removing obstacle boxes to planning scene
        self.shelf_description = {
            ## name : [x, y, z, orient.x, orient.y, orient.z, orient.w, dim_x, dim_y, dim_z]
            "base": [0.27+(0.55/2), 0.0, 0.01/2, 1.0, 0.0, 0.0, 0.0, 0.55, 1.08, 0.01],
            "shelf_1": [0.22 + (0.6/2), -0.245-0.145, 0.02, 1.0, 0.0, 0.0, 0.0, 0.6, 0.29, 0.01],
            "shelf_2": [0.22 + (0.6/2), -0.245-0.145, 0.02 + 0.28, 1.0, 0.0, 0.0, 0.0, 0.6, 0.29, 0.01],
            "shelf_3": [0.22 + (0.6/2), -0.245-0.145, 0.02 + 0.28 + 0.28, 1.0, 0.0, 0.0, 0.0, 0.6, 0.29, 0.01],
            "shelf_4": [0.82, -0.245-0.145, 0.02 + 0.3, 1.0, 0.0, 0.0, 0.0, 0.01, 0.29, 0.6],
            "shelf_5": [0.22, -0.245-0.145, 0.02 + 0.3, 1.0, 0.0, 0.0, 0.0, 0.01, 0.29, 0.6],
            "rrt_kliye_1": [0.22, +0.265+0.145, 0.02 + 0.3, 1.0, 0.0, 0.0, 0.0, 0.01, 0.39, 1.0],
            "drift_kliye_1": [0.22 + (0.6/2), +0.265, 0.02 + 0.27, 1.0, 0.0, 0.0, 0.0, 0.6, 0.29, 0.01]
        }
        
    def shelf_as_collision_boxes(self):
        collision_boxes = []
        for name, params in self.shelf_description.items():
            pose = PoseStamped()
            pose.header.frame_id = "panda_link0"
            pose.pose.position.x = params[0]
            pose.pose.position.y = params[1]
            pose.pose.position.z = params[2]
            pose.pose.orientation.x = params[3]
            pose.pose.orientation.y = params[4]
            pose.pose.orientation.z = params[5]
            pose.pose.orientation.w = params[6]
            
            collision_boxes.append((name, pose, [params[7], params[8], params[9]]))
        
        return collision_boxes
        
        # base_pose = PoseStamped()
        # base_pose.header.frame_id = "panda_link0"
        # base_pose.pose.position.x = 0.27+(0.55/2)
        # base_pose.pose.position.y = 0.0
        # base_pose.pose.position.z = 0.01/2
        # base_pose.pose.orientation.x = 1.0
        # base_pose.pose.orientation.y = 0.0
        # base_pose.pose.orientation.z = 0.0
        # base_pose.pose.orientation.w = 0.0
        # franka_moveit.add_box("base", base_pose, [0.55, 1.08, 0.01])
        # franka_moveit.add_mesh("shelf",box_pose,"./shelf.stl",[0.2, 0.2, 0.2])

        # shelf_1 = PoseStamped()
        # shelf_1.header.frame_id = "panda_link0"
        # shelf_1.pose.position.x = 0.22 + (0.6/2)
        # shelf_1.pose.position.y = -0.245-0.145
        # shelf_1.pose.position.z = 0.02
        # shelf_1.pose.orientation.x = 1.0
        # shelf_1.pose.orientation.y = 0.0
        # shelf_1.pose.orientation.z = 0.0
        # shelf_1.pose.orientation.w = 0.0
        # franka_moveit.add_box("shelf_1", shelf_1, [0.6, 0.29, 0.01])

        # shelf_2 = PoseStamped()
        # shelf_2.header.frame_id = "panda_link0"
        # shelf_2.pose.position.x = 0.22 + (0.6/2)
        # shelf_2.pose.position.y = -0.245-0.145
        # shelf_2.pose.position.z = 0.02 + 0.28
        # shelf_2.pose.orientation.x = 1.0
        # shelf_2.pose.orientation.y = 0.0
        # shelf_2.pose.orientation.z = 0.0
        # shelf_2.pose.orientation.w = 0.0
        # franka_moveit.add_box("shelf_2", shelf_2, [0.6, 0.29, 0.01])

        # shelf_3 = PoseStamped()
        # shelf_3.header.frame_id = "panda_link0"
        # shelf_3.pose.position.x = 0.22 + (0.6/2)
        # shelf_3.pose.position.y = -0.245-0.145
        # shelf_3.pose.position.z = 0.02 + 0.28 + 0.28
        # shelf_3.pose.orientation.x = 1.0
        # shelf_3.pose.orientation.y = 0.0
        # shelf_3.pose.orientation.z = 0.0
        # shelf_3.pose.orientation.w = 0.0
        # franka_moveit.add_box("shelf_3", shelf_3, [0.6, 0.29, 0.01])

        # shelf_4 = PoseStamped()
        # shelf_4.header.frame_id = "panda_link0"
        # shelf_4.pose.position.x = 0.82
        # shelf_4.pose.position.y = -0.245-0.145
        # shelf_4.pose.position.z = 0.02 + 0.3
        # shelf_4.pose.orientation.x = 1.0
        # shelf_4.pose.orientation.y = 0.0
        # shelf_4.pose.orientation.z = 0.0
        # shelf_4.pose.orientation.w = 0.0
        # franka_moveit.add_box("shelf_4", shelf_4, [0.01, 0.29, 0.6])

        # shelf_5 = PoseStamped()
        # shelf_5.header.frame_id = "panda_link0"
        # shelf_5.pose.position.x = 0.22
        # shelf_5.pose.position.y = -0.245-0.145
        # shelf_5.pose.position.z = 0.02 + 0.3
        # shelf_5.pose.orientation.x = 1.0
        # shelf_5.pose.orientation.y = 0.0
        # shelf_5.pose.orientation.z = 0.0
        # shelf_5.pose.orientation.w = 0.0
        # franka_moveit.add_box("shelf_5", shelf_5, [0.01, 0.29, 0.6])
        
        #NOTE: Abhishek: Code to define 6 positions
        
        # NOTE: Position 1 (Top shelf left most)
        # joints: 
        # [ 0.04932081  0.41680444 -0.28264505 -1.80764826 -1.65895955  1.76036052
        # -0.13009759]
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.61502854 -0.31966704  0.38818833]
        # Rot: [[-0.01092505  0.99991541 -0.00552877]
        # [-0.06773011 -0.00625634 -0.99768402]
        # [-0.99763421 -0.01052529  0.06779403]]
        # Qtn: [ 0.51249699  0.48154368  0.48395672 -0.52080575]
        # from franka_tool to world
        
        # NOTE: Position 2 (Top shelf middle)
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.4496459  -0.32577249  0.37486133]
        # Rot: [[-0.0159875   0.99918275 -0.03686478]
        # [-0.04397288 -0.0375361  -0.99832728]
        # [-0.99889516 -0.01433971  0.04453791]]
        # Qtn: [-0.4977437  -0.49421483 -0.48319599  0.52394215]
        # from franka_tool to world
        # Enter a number: 2
        # joints: 
        # [ 0.56087855  0.0201718  -0.83533652 -2.35995647 -1.76507356  1.71175639
        # 0.02144443]
        
        # NOTE: Position 3 (Top shelf right)
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.30685658 -0.31731813  0.38489819]
        # Rot: [[-0.08437571  0.99554888 -0.04176058]
        # [ 0.01714317 -0.04045304 -0.99903435]
        # [-0.99627687 -0.08501014 -0.01365388]]
        # Qtn: [-0.46408526 -0.49237026 -0.5141927   0.5270614 ]
        # from franka_tool to world
        # Enter a number: 2
        # joints: 
        # [ 0.0267715  -0.48488081 -0.38927448 -2.78192472 -1.88040301  1.66381664
        # -0.12541168]
        
        # NOTE: Position 4 (Bottom shelf left)
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.62007213 -0.29471126  0.09797535]
        # Rot: [[-0.06024339  0.99798348  0.01950539]
        # [-0.06227433  0.01574487 -0.99793483]
        # [-0.99622959 -0.06133366  0.0612014 ]]
        # Qtn: [ 0.50415843  0.46443792  0.50367847 -0.52575626]
        # from franka_tool to world
        # Enter a number: 2
        # joints: 
        # [ 0.15417526  1.00905677 -0.36440428 -1.62995756 -1.50209554  1.81501208
        # 0.19394939]
        
        # NOTE: Position 5 (Bottom shelf middle)
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.46596075 -0.29291366  0.0993864 ]
        # Rot: [[-0.01725819  0.99810282 -0.05893836]
        # [ 0.03908043 -0.05822814 -0.99753802]
        # [-0.99907739 -0.01951904 -0.03800211]]
        # Qtn: [ 0.4707737   0.5193679   0.49925268 -0.50928049]
        # from franka_tool to world
        # Enter a number: 2
        # joints: 
        # [ 0.63118503  0.94457377 -0.83804126 -2.16908984 -1.25600768  2.19826817
        # 0.40085538]


        # NOTE: Position 6 (Bottom shelf right) Might be problematic, 
        # NOTE: hit virtual wall multiple times in guide mode
        # Enter a number: 1
        # pose: 
        # Tra: [ 0.31284843 -0.29916375  0.08518446]
        # Rot: [[-0.0108778   0.99990185 -0.00766263]
        # [ 0.06284333 -0.00696424 -0.99799907]
        # [-0.99795448 -0.01133758 -0.06276261]]
        # Qtn: [ 0.47942084  0.514497    0.51640008 -0.48864083]
        # from franka_tool to world
        # Enter a number: 2
        # joints: 
        # [ 0.09193547  0.55592332 -0.35134907 -2.57623927 -1.769613    1.81996532
        # 0.77395573]

