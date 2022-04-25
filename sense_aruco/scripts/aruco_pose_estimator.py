#!/usr/bin/env python

from sense_aruco.aruco_estimator import ArucoMarkerEstimator
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, \
    PoseWithCovarianceStamped
import message_filters
import numpy as np
import rospy
import tf.transformations
from sensor_msgs.msg import Image


class ArucoPoseEstimatorNode:

    def __init__(self, paramfilepath, family_name, marker_side_len,
        frame_of_marker="/map", compute_covar=False):
        self.cvbridge = CvBridge()

        # Set up the marker estimator.
        self.marker_estimator = ArucoMarkerEstimator(
            paramfilepath=paramfilepath, family_name=family_name,
            marker_side_len=marker_side_len)

        self.compute_covar = compute_covar
        self.__frame_of_marker = frame_of_marker
    #end def

    def callback(self, color_image, depth_data):
        """
        Callback function used to estimate the marker pose and predicted
        uncertainty.
        """
        # Get the markers from the input image.
        marker_list = self.get_markers(color_image)

        # Extract the poses, transform them from camera to global coords,
        # and publish the messages.
        for marker in marker_list:
            pose = marker['pose']

            # TODO: perform the transformation from camera to global coords.

            qx, qy, qz, qw = tf.transformations.quaternion_from_matrix(pose)
            tx, ty, tz = pose[:3, 3].flatten()

            ros_stamp = color_image.header.stamp
            ros_pose = Pose(position = Point(x=tx, y=ty, z=tz),
                orientation = Quaternion(x=qx, y=qy, z=qz, w=qw))

            msg = None
            if self.compute_covar:
                # TODO: neural network predictions here.

                # Form the covariance matrix.
                covar = np.eye(6)
                covar[[2,3,4],[2,3,4]] = 0.
                covar = covar.flatten()

                msg = PoseWithCovarianceStamped()
                msg.header.stamp = ros_stamp
                msg.header.frame_id = self.__frame_of_marker
                msg.pose.pose = ros_pose
                msg.pose.covariance = covar.tolist()

            else:
                msg = PoseStamped()
                msg.header.stamp = ros_stamp
                msg.header.frame_id = self.__frame_of_marker
                msg.pose = ros_pose
            #end if

            self.pose_pub.publish(msg)
        #end for
    #end def

    def get_markers(self, image_msg):
        # Convert the color image to a CV image, and then get the data.
        cv_image = self.cvbridge.imgmsg_to_cv2(image_msg,
            desired_encoding='bgr8')
        return self.marker_estimator.detect(cv_image, get_pose=True)
    #end def

    def setup_node(self, publisher_topic, queue_len):
        self.color_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber(
            '/camera/depth/image_raw', Image)

        if self.compute_covar:
            self.pose_pub = rospy.Publisher(publisher_topic,
                PoseWithCovarianceStamped, queue_size=queue_len)
        else:
            self.pose_pub = rospy.Publisher(publisher_topic,
                PoseStamped, queue_size=queue_len)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_len, 0.01)

        self.ts.registerCallback(self.callback)
#end class

if __name__ == '__main__':
    queue_len = 60
    rospy.init_node('aruco_pose_estimator', anonymous=True)
    
    # Configure the node.
    paramfilepath = rospy.get_param("~parampath")
    family_name = rospy.get_param("~familyname", "DICT_4X4_1000")
    marker_side_len = float(rospy.get_param("~marker_len"))
    compute_covar = bool(rospy.get_param("~compute_covar", False))
    publisher_topic = rospy.get_param("~pose_topic", "aruco_marker")

    aruco_estimator_node = ArucoPoseEstimatorNode(paramfilepath=paramfilepath,
        family_name=family_name, marker_side_len=marker_side_len,
        compute_covar=compute_covar)
    
    aruco_estimator_node.setup_node(publisher_topic, queue_len)
    
    rospy.spin()
#end if
