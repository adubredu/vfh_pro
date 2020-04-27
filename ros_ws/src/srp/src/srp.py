#!/usr/bin/env python
import rospy
import copy
import actionlib
from math import sin, cos, sqrt
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_test import MoveFetch
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()


# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()



# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.robot_pose = Pose()
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odometry_handler)
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def odometry_handler(self, pose):
        self.robot_pose = pose.pose 

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        rp = self.robot_pose.pose.position
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = rp.x+x
        goal.target.point.y =  rp.y+y
        goal.target.point.z =  rp.z+z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()




# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")
        # self.group = moveit_commander.MoveGroupCommander("arm")

        self.green_pose = None
        self.red_pose = None
        self.violet_pose = None
        self.blue_pose = None
        self.brown_pose = None
        self.yellow_pose = None
        self.black_pose = None

        rospy.Subscriber('/green_cube',Pose,self.green_cb)
        rospy.Subscriber('/red_cube',Pose,self.red_cb)
        rospy.Subscriber('/violet_cube',Pose,self.violet_cb)
        rospy.Subscriber('/blue_cube',Pose,self.blue_cb)
        rospy.Subscriber('/brown_cube',Pose,self.brown_cb)
        rospy.Subscriber('/yellow_cube',Pose,self.yellow_cb)
        rospy.Subscriber('/black_cube',Pose,self.black_cb)

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def green_cb(self,data):
        self.green_pose = data
    def red_cb(self,data):
        self.red_pose = data
    def violet_cb(self,data):
        self.violet_pose = data
    def blue_cb(self,data):
        self.blue_pose = data
    def brown_cb(self,data):
        self.brown_pose = data
    def yellow_cb(self,data):
        self.yellow_pose = data
    def black_cb(self,data):
        self.black_pose = data

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0])

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0])

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        # print(self.objects)
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue

            # check size
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None


    def get_dist_btn(self,source, target):
        sx = source.position.x; sy = source.position.y; sz = source.position.z; 
        tx = target.position.x; ty = target.position.y; tz = target.position.z;

        dist = sqrt((sx-tx)**2 + (sy-ty)**2 + (sz-tz)**2)
        return dist


    def getTargetObject(self,name):
        if len(self.objects) == 0:
            return None, None

        obj = self.green_pose
        if name == 'green':
            obj = self.green_pose
        elif name == 'red':
            obj = self.red_pose
        elif name == 'brown':
            obj = self.brown_pose
        elif name == 'violet':
            obj = self.violet_pose
        elif name == 'yellow':
            obj = self.yellow_pose
        elif name == 'blue':
            obj = self.blue_pose
        elif name == 'black':
            obj = self.black_pose

        mindist = 100
        goalpose = self.objects[0]
        for target in self.objects:
            dist = self.get_dist_btn(target.object.primitive_poses[0], obj)
            if dist <= mindist:
                mindist = dist
                goalpose = target

        return goalpose.object, goalpose.grasps 



    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


    def raise_arm(self,pose):
        pose.position.z+=0.5
        move_fetch_obj = MoveFetch()
        move_fetch_obj.move_manager(pose_requested=pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")

    def move_arm_aside(self):
        arm_joint_positions_L = [-1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
        arm_joint_positions_R = [0.8, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
        move_fetch_obj = MoveFetch()
        rate_obj = rospy.Rate(1)
        move_fetch_obj.move_manager(pose_requested=None,
                                    joints_array_requested=arm_joint_positions_R,
                                    movement_type_requested="JOINTS")

    def move_arm_to_pregrasp(self):
        arm_joint_positions= [0, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
        move_fetch_obj = MoveFetch()
        rate_obj = rospy.Rate(1)
        move_fetch_obj.move_manager(pose_requested=None,
                                    joints_array_requested=arm_joint_positions,
                                    movement_type_requested="JOINTS")

    def open_or_close_gripper(self,state):
        move_fetch_obj = MoveFetch()
        max_effort = 10.0
        if state == 'open':
            grip_position = 0.5
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[
                                            grip_position, max_effort],
                                        movement_type_requested="GRIPPER")
        elif state == 'close':
            grip_position = 0.02
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[
                                            grip_position, max_effort],
                                        movement_type_requested="GRIPPER")


    def move_arm_to_position(self,pose):
        move_fetch_obj = MoveFetch()
        move_fetch_obj.move_manager(pose_requested=pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")





if __name__=='__main__':
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    grasping_client.open_or_close_gripper('open')
    grasping_client.tuck()
    # move_base.goto(-6.520,-3.732,0.0)

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])

    # Point the head at the cube we want to pick
    head_action.look_at(0,-1.0,0., "map")

    # '''
    # Get block to pick
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene()
        cube, grasps = grasping_client.getTargetObject('black')
        if cube == None:
            rospy.logwarn("Perception failed.")
            continue

    #     # Pick the block
        if grasping_client.pick(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")



    # Tuck the arm
    grasping_client.raise_arm(cube.primitive_poses[0])
    grasping_client.move_arm_aside()

 
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = cube.primitive_poses[0]
        pose.pose.position.z += 0.05
        pose.header.frame_id = cube.header.frame_id
        if grasping_client.place(cube, pose):
            break
        rospy.logwarn("Placing failed.")

   
    # '''