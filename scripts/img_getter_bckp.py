import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import aruco_pose

bridge = CvBridge()
hash_calc = cv2.img_hash.BlockMeanHash_create()
hash_value = np.expand_dims(np.ndarray([1]), 0)
callback_count = 0

def calculate_pose(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    curr_hash = hash_calc.compute(cv_image)
    global hash_value, callback_count
    if (curr_hash == hash_value).all():
        return
    
    #print('Frame changed.')
    hash_value = curr_hash
    aruco_pose.main(cv_image)
    


def listener():
    rospy.init_node('img_subscriber', anonymous=True)
    rospy.Subscriber("/free_camera/image_raw", Image, calculate_pose)
    rospy.spin()

if __name__ == '__main__':
    listener()

