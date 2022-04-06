import rospy
from geometry_msgs.msg import PoseStamped

def callback(pose_stamp):
    try:
        print('Heard:', pose_stamp.pose.position.x)
    except Exception as E:
        pass

def listener():
    rospy.init_node('pose_subscriber', anonymous=True)
    rospy.Subscriber('/pose_estimate', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()