from gazebo_msgs.srv import GetModelState
import rospy
from aruco_pose import euler_from_quaternion
import math

def rad_2_deg(rad):
    return rad*180/math.pi

model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
prev_x=0
prev_y=0
prev_z=0
while(True):
  resp_coordinates = model_coordinates('aruco_marker', 'camera')
#resp_coordinates = model_coordinates('camera', 'world')
#resp_coordinates = model_coordinates('aruco_marker', 'world')
  curr_x = resp_coordinates.pose.position.x
  curr_y = resp_coordinates.pose.position.y
  curr_z = resp_coordinates.pose.position.z
  if (prev_x != curr_x)or(prev_y != curr_y)or(prev_z != curr_z):
      print(curr_x, curr_y,curr_z)
      prev_x = curr_x
      prev_y = curr_y
      prev_z = curr_z

euler_angles = euler_from_quaternion(resp_coordinates.pose.orientation.x, resp_coordinates.pose.orientation.y, resp_coordinates.pose.orientation.z, resp_coordinates.pose.orientation.w)

print('Ground Truth')
print(rad_2_deg(euler_angles[0]), rad_2_deg(euler_angles[1]), rad_2_deg(euler_angles[2]))
