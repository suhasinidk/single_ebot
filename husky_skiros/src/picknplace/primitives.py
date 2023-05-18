#!/usr/bin/env python

from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import MoveGroupActionFeedback
from skiros2_common.core.abstract_skill import State
from skiros2_common.core.primitive import PrimitiveBase

from skiros2_common.core.conditions import ConditionProperty, ConditionRelation
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_skill.core.skill import SkillDescription

import math
import moveit_commander
import rospy
import sys

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# Primitives
class ArmToHomePrimitiveDescription(SkillDescription):
    def createDescription(self):
        pass

# class PickObjectPrimitiveDescription(SkillDescription):
#     def createDescription(self):
#         # self.addParam('Enter object no:', str, ParamTypes.Required)
#         pass

# class PlaceObjectPrimitiveDescription(SkillDescription):
#     def createDescription(self):
#         # self.addParam('Place', 0, ParamTypes.Required)
#         pass

class ArmToHomePrimitive(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ArmToHomePrimitiveDescription(), self.__class__.__name__)

    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.mi_cmdr = moveit_commander.MoveGroupCommander('ur_and_gripper')
        rospy.Subscriber('/move_group/status', GoalStatusArray, self.feedback)

    def onStart(self):
        self.mi_cmdr.set_joint_value_target(self.mi_cmdr.get_named_target_values('home'))
        self.mi_cmdr.go(wait=False)
        self.moving = True
        return True

    def execute(self):
        if self.moving:
            return self.step('Moving arm to Home position')
        return self.success('Finished moving arm to Home position')

    def onPreempt(self):
        self.mi_cmdr.stop()
        self.mi_cmdr.clear_pose_targets()
        return self.success('Moving arm to Home position preempted')

    def feedback(self, msg):
        self.moving = any(s.status in (0, 1) for s in msg.status_list)

# class PickObjectPrimitive(PrimitiveBase):
#     def attach_obj(self):
#         attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
#                                             Attach)
#         attach_srv.wait_for_service()
        
#         req = AttachRequest()
#         req.model_name_1 = "ebot"
#         req.link_name_1 = "gripper_finger2_finger_tip_link"
#         req.model_name_2 = "box2"
#         req.link_name_2 = "box_link"
        
#         attach_srv.call(req)

#     def createDescription(self):
#         self.setDescription(PickObjectPrimitiveDescription(), self.__class__.__name__)

#     def onInit(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         self.mi_cmdr = moveit_commander.MoveGroupCommander('ur5_bot')
#         rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.feedback)

#     def onStart(self):
#         open = 1.0
#         self.arm_fail = False
#         # self.move = True
#         # return True

#         self.mi_cmdr.set_joint_value_target(self.mi_cmdr.get_named_target_values('pick'))
#         self.mi_cmdr.go(wait=False)
#         self.moving = True
#         rospy.sleep(5)

#         return True

#     def execute(self):
#         close =0.45
        
#         if self.arm_fail:
#             return self.fail('Failed to load paint (unable to plan arm motion)', -1)
         
#         joint_goal = self.mi_cmdr.get_current_joint_values()
#         joint_goal[6] = close
#         self.mi_cmdr.set_joint_value_target(joint_goal)
#         self.mi_cmdr.go(wait=False)
#         rospy.sleep(10)
#         self.attach_obj()

#         carry_goal = self.mi_cmdr.get_named_target_values('carry')
#         carry_goal['gripper_finger1_joint'] = close
#         self.mi_cmdr.set_joint_value_target(carry_goal)
#         self.mi_cmdr.go(wait=False)
#         rospy.sleep(2)

#         return self.success('Finished')

#     def onPreempt(self):
#         self.mi_cmdr.stop()
#         self.mi_cmdr.clear_joint_angles()
#         return self.success('Loading paint preempted')

#     def feedback(self, msg):
#         if self.state != State.Running:
#             return

#         if msg.status.status == 3:
#             # self.p += 1
#             self.move = True
#         elif msg.status.status == 4:
#             self.arm_fail = True

# class PlaceObjectPrimitive(PrimitiveBase):
#     def detach_obj(self):
#         attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
#                                         Attach)
#         attach_srv.wait_for_service()

#         req = AttachRequest()
#         req.model_name_1 = "ebot"
#         req.link_name_1 = "gripper_finger2_finger_tip_link"
#         req.model_name_2 = "box2"
#         req.link_name_2 = "box_link"

#         attach_srv.call(req)
        
#     def createDescription(self):
#         self.setDescription(PlaceObjectPrimitiveDescription(), self.__class__.__name__)

#     def onInit(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         self.mi_cmdr = moveit_commander.MoveGroupCommander('ur5_bot')
#         rospy.Subscriber('/move_group/feedback', MoveGroupActionFeedback, self.feedback)

#     def onStart(self):
#         close =0.45

#         carry_goal = self.mi_cmdr.get_named_target_values('carry')
#         carry_goal['gripper_finger1_joint'] = close
#         self.mi_cmdr.set_joint_value_target(carry_goal)
#         self.mi_cmdr.go(wait=False)
#         rospy.sleep(4)

#         self.moving = True
#         self.arm_fail = False
#         self.move = True
#         return True

#     def execute(self):
#         open = 0.0
#         close =0.45

#         carry_goal = self.mi_cmdr.get_named_target_values('place')
#         carry_goal['gripper_finger1_joint'] = close
#         self.mi_cmdr.set_joint_value_target(carry_goal)
#         self.mi_cmdr.go(wait=False)
#         rospy.sleep(8)

#         carry_goal = self.mi_cmdr.get_named_target_values('place')
#         carry_goal['gripper_finger1_joint'] = open
#         self.mi_cmdr.set_joint_value_target(carry_goal)
#         self.mi_cmdr.go(wait=False)
#         self.detach_obj()
#         rospy.sleep(2)

#         if self.moving:
#             return self.step('Moving arm')
#         return self.success('Finished moving arm')


#     def onPreempt(self):
#         self.mi_cmdr.stop()
#         self.mi_cmdr.clear_pose_targets()
#         return self.success('Placing object preempted')

#     def feedback(self, msg):
#         if self.state != State.Running:
#             return

#         if msg.status.status == 3:
#             self.move = True
#         elif msg.status.status == 4:
#             self.arm_fail = True