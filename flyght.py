from __future__ import print_function
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

flag=False
l = []
def foo_cal(data):
    global flag
    print(data.data[0])
    l.append(data.data[0])
    flag=True


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

# navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)

coords = {1:[10,10,1],
          2:[0,10,1],
          3:[10,0,1],
          4:[5,5,1],
          5:[8,5,1],
          6:[5,8,1]}

path = [1,2,3,4,5,6]

for i in path:
    navigate_wait(x=coords[i][0], y=coords[i][1], z=coords[i][2], speed=1, frame_id='aruco_map')
    r = rospy.Subscriber('sensors', Float32MultiArray, foo_cal)
    while not flag:
        rospy.sleep(0.05)
    r.unregister()
    flag = False

print(l)
navigate_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')


# land()
