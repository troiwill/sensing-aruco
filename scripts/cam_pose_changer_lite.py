import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import aruco_pose

bridge = CvBridge()
hash_calc = cv2.img_hash.BlockMeanHash_create()
hash_value = np.expand_dims(np.ndarray([1]), 0)

def calculate_pose(data):
    global hash_value, subscriber
    subscriber.unregister()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    curr_hash = hash_calc.compute(cv_image)
    if (curr_hash == hash_value).all():
        return

    hash_value = curr_hash
    aruco_pose.main(cv_image)

freq = 1 #hz

def main():
    rospy.init_node('set_pose')
    rospy.init_node('img_subscriber', anonymous=True)
    pos_z=1.1
    while not rospy.is_shutdown():
      for deg_x in np.linspace(1.2981, 1.7727, 7):#range (-0.5236, 0.5236, 10):
        #rot = Rotation.from_euler('xyz', [0, 90, 90], degrees=True)
        rot = Rotation.from_euler('xyz', [1.56, deg_x, 1.56], degrees=False)
        #deg_x += .1745
        rot_quat = rot.as_quat()
        state_msg = ModelState()
        state_msg.model_name = 'camera'
        state_msg.pose.position.x = -0.1
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = pos_z
        #pos_z += .1
        state_msg.pose.orientation.x = rot_quat[0]
        state_msg.pose.orientation.y = rot_quat[1]
        state_msg.pose.orientation.z = rot_quat[2]
        state_msg.pose.orientation.w = rot_quat[3]
        rate = rospy.Rate(freq)

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
            global subscriber
            subscriber = rospy.Subscriber("/free_camera/image_raw", Image, calculate_pose)
            print('changed state')
            rate.sleep()

        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

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
