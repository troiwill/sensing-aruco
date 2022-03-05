from math import sqrt
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import aruco_pose
from gazebo_msgs.srv import GetModelState
from aruco_pose import euler_from_quaternion
import os
import pickle

bridge = CvBridge()
hash_calc = cv2.img_hash.BlockMeanHash_create()
hash_value = np.expand_dims(np.ndarray([1]), 0)
callback_count = 0

distances = []#np.array([])
gt_distances = []#np.array([])
#a_file1 = open("gt.txt", "wb")

model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
resp_coordinates = model_coordinates('aruco_marker', 'camera')

def calculate_pose(data):
    #subscriber.unregister()
    #print('entered')
    #if not os.path.exists('flag.txt'):
    #    return
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    curr_hash = hash_calc.compute(cv_image)
    global hash_value, callback_count
    if (curr_hash == hash_value).all():
        return
    
    print('Frame changed.')
    hash_value = curr_hash
    x, y, z, r, p, yaw = aruco_pose.main(cv_image)
    print('Prediction')
    print(r, p, yaw)
    distance = sqrt(x**2 + y**2 + z**2)
    #np.append(distances, distance)
    distances.append(distance)
    
    resp_coordinates = model_coordinates('aruco_marker', 'camera')
    gt_distance = sqrt(resp_coordinates.pose.position.x**2 + resp_coordinates.pose.position.y**2 + resp_coordinates.pose.position.z**2)
    #np.append(gt_distances, gt_distance)
    gt_distances.append(gt_distance)

    with open("estimates.txt", "wb") as a_file:
        pickle.dump(distances, a_file)
    with open("gt.txt", "wb") as a_file1:
        pickle.dump(gt_distances, a_file1)
    #np.save(a_file, distances)#, allow_pickle=True)
    #np.save(a_file1, gt_distances)#, allow_pickle=True)
    callback_count += 1
    print(callback_count)


def listener():
    rospy.init_node('img_subscriber', anonymous=True)
    rospy.Subscriber("/free_camera/image_raw", Image, calculate_pose)
    rospy.spin()

if __name__ == '__main__':
    listener()

