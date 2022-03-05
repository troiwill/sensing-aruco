import time
import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation
import numpy as np
from img_getter import *
import os
import math

freq = 10 #hz
mid_deg_x = 1.57
half_deg_range = math.radians(50)/2
pos_z = 0.6
def main():
    rospy.init_node('set_pose')

    #while not rospy.is_shutdown():
    for pos_z in np.linspace(0.6, 7, 21):
      for deg_x in np.linspace(mid_deg_x-half_deg_range, mid_deg_x+half_deg_range, 10):#range (-0.5236, 0.5236, 10):
        rot = Rotation.from_euler('xyz', [1.56, deg_x, 1.56], degrees=False)
        rot_quat = rot.as_quat()
        state_msg = ModelState()
        state_msg.model_name = 'camera'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = pos_z
        state_msg.pose.orientation.x = rot_quat[0]
        state_msg.pose.orientation.y = rot_quat[1]
        state_msg.pose.orientation.z = rot_quat[2]
        state_msg.pose.orientation.w = rot_quat[3]
        rate = rospy.Rate(freq)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            file1 = open('flag.txt', 'w+')
            resp = set_state( state_msg )
            time.sleep(0.03)
            os.remove('flag.txt')
            print('changed state')
            rate.sleep()

        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


'''
import rospy
from gazebo_msgs.msg import ModelState  #import String

def talker():
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pose_str = ['camera', '{position: {x: 1, y: 0, z: 2}, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099}}', '{linear: {x: 0, y: 0, z: 0}, angular:{x: 0, y: 0, z: 0.1}}', 'world']
        rospy.loginfo(pose_str)
        pub.publish(pose_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
'''
