from __future__ import print_function
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import sys
import threading
import os

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from threading import Thread

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

class VideoRecorder:
    def __init__(self):
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if not os.path.exists(os.environ['HOME']+"/L22_AERO_LOG"):
            os.mkdir(os.environ['HOME']+"/L22_AERO_LOG")
        self.UPDATE_RATE = 5
        self.video_writer = cv2.VideoWriter(os.environ['HOME']+"/L22_AERO_LOG/LOG_IMAGE_RAW_real_drone.avi", self.fourcc, self.UPDATE_RATE, (320, 240))
        self.image_raw_frame = np.zeros((240, 320, 3), dtype="uint8")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/main_camera/image_raw_throttled", Image, self.img_clb)
        self.stopped = False
    
    def img_clb(self, msg):
        self.image_raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def start(self):    
        Thread(target=self.videowriter, args=()).start()
        return self

    def videowriter(self):
        try:
            r = rospy.Rate(self.UPDATE_RATE)
            while not self.stopped:
                self.video_writer.write(self.image_raw_frame)
                r.sleep()
        except KeyboardInterrupt:
            self.video_writer.release()
            self.stopped = True
    def stop(self):
        self.stopped = True
        self.video_writer.release()






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

vr = VideoRecorder().start()

coords = {1:[10,10,1],
          2:[0,10,1],
          3:[10,0,1],
          4:[5,5,1],
          5:[8,5,1],
          6:[5,8,1]}

path = [1,2]

for i in path:
    navigate_wait(x=coords[i][0], y=coords[i][1], z=coords[i][2], speed=1, frame_id='aruco_map')
    r = rospy.Subscriber('sensors', Float32MultiArray, foo_cal)
    while not flag:
        rospy.sleep(0.05)
    r.unregister()
    flag = False

print(l)
navigate_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
vr.stop()

# land()
