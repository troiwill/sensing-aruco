import message_filters
from math import sqrt
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import aruco_pose
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from aruco_pose import euler_from_quaternion
import os
import pickle

bridge = CvBridge()
# hash_calc = cv2.img_hash.BlockMeanHash_create()
# hash_value = np.expand_dims(np.ndarray([1]), 0)
# hash_value_depth = np.expand_dims(np.ndarray([1]), 0)

def calculate_pose(data):
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    # curr_hash = hash_calc.compute(cv_image)
    # global hash_value 
    # if (curr_hash == hash_value).all():
    #     pose_estimate = PoseStamped()
    #     pose_estimate.pose.position.x = -1
    #     return pose_estimate
    # hash_value = curr_hash
    x, y, z, q_x, q_y, q_z, q_w = aruco_pose.main(cv_image)
    # print('*** Predicted pose ***')
    # print('x:', x, 'y:', y, 'z:', z, 'roll:', r, 'pitch:', p, 'yaw:', yaw)
    pose_stamp = PoseStamped()
    pose_stamp.pose = Pose(position = Point(x=x, y=y, z=z),
                           orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w))
    pose_stamp.header =  data.header.stamp
    return pose_stamp



def calculate_depth(data):
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    roi_center = [360, 640] #row, col possible values for row & col: (0 to num_rows-1) & (0 to num_cols-1)
    roi_width = 50
    roi_height = 50
    roi_bounds = [[roi_center[0]-roi_height//2, roi_center[1]-roi_width//2], #ul
                  [roi_center[0]+roi_height//2, roi_center[1]-roi_width//2], #ll
                  [roi_center[0]+roi_height//2, roi_center[1]+roi_width//2], #lr
                  [roi_center[0]-roi_height//2, roi_center[1]+roi_width//2]] #ur
    roi_bounds = np.array(roi_bounds).astype(int)
    assert (np.min(roi_bounds[:, 0]) >= 0) and (np.max(roi_bounds[:, 0]) < depth_image.shape[0]) and \
           (np.min(roi_bounds[:, 1]) >= 0) and (np.max(roi_bounds[:, 1]) < depth_image.shape[1]),'infeasible roi is defined'

    # curr_hash = np.array([depth_image[0], depth_image[int(depth_image.shape[0]/2)], depth_image[-1]])
    # global hash_value_depth
    # if (np.equal(curr_hash, hash_value_depth).all()):
    #     return
    # hash_value_depth = curr_hash
    
    median_depth = np.median(depth_image[roi_bounds[0][0]:roi_bounds[1][0], roi_bounds[0][1]:roi_bounds[3][1]])
    print('\n*** Median Depth ***\n')
    print(median_depth, 'mm')
    return
    
def callback(color_data, depth_data, pose_pub, publish_pose=True):
    pose_estimate = calculate_pose(color_data)
    # calculate_depth(depth_data)
    # if publish_pose:
    #     if pose_estimate.pose.position.x != -1:
    #         print(pose_estimate.pose.position.x)
            pose_pub.publish(pose_estimate)

def node():
    rospy.init_node('aruco_pose_estimator', anonymous=True)
    color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 10, 0.01)
    pose_pub = rospy.Publisher('pose_estimate', PoseStamped, queue_size=10)

    ts.registerCallback(callback, (pose_pub))


    rospy.spin()

if __name__ == '__main__':
    node()
