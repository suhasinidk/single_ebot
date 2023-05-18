#!/usr/bin/env python3

import rospy, time
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler
import random
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import rospkg
print ('******************************************')
rospack = rospkg.RosPack()
path = rospack.get_path('ebot_gazebo')
print (path)
sdf_model = open(path + '/models/coke_can/model.sdf', 'r').read()
sdf_model2 = open(path + '/models/glue/model.sdf', 'r').read()
sdf_model3 = open(path + '/models/soap2/model.sdf', 'r').read()

def create_model_request(sdf_model, modelname, px, py, pz, rr, rp, ry):
    """Create a SpawnModelRequest with the parameters of the model given.
    modelname: name of the model for gazebo
    px py pz: position of the model (and it's collision model)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    sx sy sz: size of the model"""
    model = deepcopy(sdf_model)
    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = model
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz
    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]
    return req


def ebot_brake():
    rospy.init_node('task4_gazebo_sim')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo('Applying brakes to ebot')
    req = AttachRequest()
    req.model_name_1 = 'ebot'
    req.link_name_1 = 'ebot_base'
    req.model_name_2 = 'ground_plane'
    req.link_name_2 = 'link'
    attach_srv.call(req)


def random_model_pose(p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):
    x_position = [
     p1_x, p2_x, p3_x]
    y_position = [p1_y, p2_y, p3_y]
    random.shuffle(x_position)
    random.shuffle(y_position)
    i = round(random.uniform(0, 3.14), 2)
    pose1 = [
     x_position[0], y_position[0], 0.87, 0.0, 0.0, 0.0]
    pose2 = [x_position[1], y_position[1], 0.87, 0.0, 0.0, 0.0]
    pose3 = [x_position[2], y_position[2], 0.87, 0.0, 0.0, 1.053]
    return (
     pose1, pose2, pose3)


if __name__ == '__main__':
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo('Waiting for /gazebo/spawn_sdf_model service...')
    spawn_srv.wait_for_service()
    rospy.loginfo('Connected to service!')
    pose1, pose2, pose3 = random_model_pose(8.108564, 3.239278, 7.971279, 3.393995, 7.84, 3.24)
    rospy.loginfo('Spawning coke_can')
    req1 = create_model_request(sdf_model, 'coke_can', pose1[0], pose1[1], pose1[2], pose1[3], pose1[4], pose1[5])
    spawn_srv.call(req1)
    rospy.loginfo('Spawning box2')
    req3 = create_model_request(sdf_model3, 'battery', pose2[0], pose2[1], pose2[2], pose2[3], pose2[4], pose2[5])
    rospy.loginfo('Spawning glue')
    spawn_srv.call(req3)
    req2 = create_model_request(sdf_model2, 'glue', pose3[0], pose3[1], pose3[2], pose3[3], pose3[4], pose3[5])
    spawn_srv.call(req2)
    rospy.sleep(10.0)
    ebot_brake()
