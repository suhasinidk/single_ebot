from skiros2_skill.core.skill import SkillDescription, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.abstract_skill import State

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib
import math
import rospy
import tf
import rospy

import moveit_commander
import rospy
import sys

from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import MoveGroupActionFeedback
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

wall_num = ""

class PickObjectPrimitiveDescription(SkillDescription):
    def createDescription(self):
        # self.addParam('Enter object no:', str, ParamTypes.Required)
        pass

class PlaceObjectPrimitiveDescription(SkillDescription):
    def createDescription(self):
        # self.addParam('Place', 0, ParamTypes.Required)
        pass

#################################################################################
# NavigateToGoalPrimitive
#################################################################################

class NavigateToGoalPrimitiveDescription(SkillDescription):

    def createDescription(self):
        # self.addParam('Destination', Element('skiros:Location'), ParamTypes.Required)
        self.addParam('Enter wall no:', str, ParamTypes.Required)

class NavigateToGoalPrimitive(PrimitiveBase):

    def createDescription(self):
        self.setDescription(NavigateToGoalPrimitiveDescription(),
                            self.__class__.__name__)

    def onInit(self):
        self.mb_client = actionlib.SimpleActionClient('move_base',
                                                      MoveBaseAction)
        self.mb_client.wait_for_server()

    def onStart(self):
        global wall_num

        yaw = 0

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('map', 'ebot_base', rospy.Time(0),
                                          rospy.Duration(5))

        self.wall_loc = self.params['Enter wall no:'].value
        
        if self.wall_loc != "0" and self.wall_loc != "4" and self.wall_loc != "5" and self.wall_loc != "6":
            wall_num = self.wall_loc
        
        if self.wall_loc == "0":
            print("Going to Home")
            x_pos = 0.0
            y_pos = 0.0

        if self.wall_loc == "1":
            print("Going to wall 1")
            x_pos = -2.342495
            y_pos = 5.980561

        if self.wall_loc == "2":
            print("Going to wall 2")
            x_pos = -0.252864
            y_pos = 5.989698

        if self.wall_loc == "3":
            print("Going to wall 3")
            x_pos = 1.911049
            y_pos = 5.999160

        if self.wall_loc == "4":
            print("Going to wall 4")
            x_pos = -9.089227
            y_pos = -0.500980
            yaw = 3.14

        if self.wall_loc == "5":
            print("Going to wall 5")
            x_pos = -7.480796
            y_pos = -0.556957
            yaw = 3.14

        if self.wall_loc == "6":
            print("Going to wall 6")
            x_pos = -7.445877
            y_pos = 1.141898
            yaw = 3.14

        self.pos_prev = None
        self.dist = 0
        self.status = 1
        self.pose_start = self._get_pose()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x_pos
        self.goal.target_pose.pose.position.y = y_pos
        self.goal.target_pose.pose.orientation = Quaternion(
            *quaternion_from_euler(0, 0, yaw))
        self.mb_client.send_goal(self.goal, self._done_callback)

        return True

    def execute(self):
        pose = self._get_pose()
        if self.pos_prev:
            self.dist += math.sqrt((self.pos_prev[0] - pose[0][0])**2 +
                                   (self.pos_prev[1] - pose[0][1])**2)
        self.pos_prev = pose[0]

        msg = 'Navigating {} to {} [Pose: {}, Dist: {}]'.format(
            self.pose_start, self.goal.target_pose.pose.position, pose,
            self.dist)
        if self.status == 1:
            return self.step(msg)
        elif self.status == 3:
            return self.success(msg)
        return self.fail(msg, -2)

    def onPreempt(self):
        self.mb_client.cancel_goal()
        return self.success('Navigation preempted')

    def _done_callback(self, status, result):
        self.status = status

    def _get_pose(self):
        try:
            p, o = self.tf_listener.lookupTransform('map', 'ebot_base',
                                                    rospy.Time(0))
            return ((p[0], p[1]), euler_from_quaternion(o)[2])
        except:
            return None

class PickObjectPrimitive(PrimitiveBase):
    def attach_obj(self):
        try:
            attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                                Attach)
            attach_srv.wait_for_service()

            req = AttachRequest()
            req.model_name_1 = "ebot"
            req.link_name_1 = "wrist_3_link"
            
            self.num = wall_num

            if wall_num == "1":
                req.model_name_2 = "box1"

            if wall_num == "2":
                req.model_name_2 = "box2"

            if wall_num == "3":
                req.model_name_2 = "box3"

            req.link_name_2 = "box_link"

            attach_srv.call(req)

        except:
            pass
        
    def createDescription(self):
        self.setDescription(PickObjectPrimitiveDescription(), self.__class__.__name__)

    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.mi_cmdr = moveit_commander.MoveGroupCommander('ur5_bot')
        rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.feedback)

    def onStart(self):

        self.arm_fail = False

        self.mi_cmdr.set_joint_value_target(self.mi_cmdr.get_named_target_values('pick'))
        self.mi_cmdr.go(wait=False)
        self.moving = True

        return True

    def execute(self):
        
        if self.arm_fail:
            return self.fail('Failed to load paint (unable to plan arm motion)', -1)
         
        rospy.sleep(4)
        self.attach_obj()

        carry_goal = self.mi_cmdr.get_named_target_values('carry')
        self.mi_cmdr.set_joint_value_target(carry_goal)
        self.mi_cmdr.go(wait=False)

        return self.success('Finished')

    def onPreempt(self):
        self.mi_cmdr.stop()
        self.mi_cmdr.clear_joint_angles()
        return self.success('Loading paint preempted')

    def feedback(self, msg):
        if self.state != State.Running:
            return

        if msg.status.status == 3:
            # self.p += 1
            self.move = True
        elif msg.status.status == 4:
            self.arm_fail = True

class PlaceObjectPrimitive(PrimitiveBase):
    def detach_obj(self):
        try:
            attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                            Attach)
            attach_srv.wait_for_service()

            req = AttachRequest()
            req.model_name_1 = "ebot"
            req.link_name_1 = "wrist_3_link"

            if wall_num == "1":
                req.model_name_2 = "box1"

            if wall_num == "2":
                req.model_name_2 = "box2"

            if wall_num == "3":
                req.model_name_2 = "box3"

            req.link_name_2 = "box_link"

            attach_srv.call(req)

        except:
            pass
        
    def createDescription(self):
        self.setDescription(PlaceObjectPrimitiveDescription(), self.__class__.__name__)

    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.mi_cmdr = moveit_commander.MoveGroupCommander('ur5_bot')
        rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.feedback)

    def onStart(self):

        carry_goal = self.mi_cmdr.get_named_target_values('carry')
        self.mi_cmdr.set_joint_value_target(carry_goal)
        self.mi_cmdr.go(wait=False)

        self.moving = True
        self.arm_fail = False
        self.move = True
        return True

    def execute(self):

        carry_goal = self.mi_cmdr.get_named_target_values('place')
        self.mi_cmdr.set_joint_value_target(carry_goal)
        self.mi_cmdr.go(wait=False)
        rospy.sleep(4)

        self.detach_obj()
        rospy.sleep(2)

        return self.success('Finished moving arm')

    def onPreempt(self):
        self.mi_cmdr.stop()
        self.mi_cmdr.clear_pose_targets()
        return self.success('Placing object preempted')

    def feedback(self, msg):
        if self.state != State.Running:
            return

        if msg.status.status == 3:
            self.move = True
        elif msg.status.status == 4:
            self.arm_fail = True