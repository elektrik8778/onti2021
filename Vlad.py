import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('computer_vision_sample')
bridge = CvBridge()
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (320, 240))

def image_callback(data):
    telem = get_telemetry(frame_id='aruco_map')
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    cv2.putText(cv_image, 'y = '+str("%.1f" % telem.y), (0,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
    cv2.putText(cv_image, 'x = '+str("%.1f" % telem.x), (0,24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
    cv2.putText(cv_image, 'z = '+str("%.1f" % telem.z), (0,36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
    out.write(cv_image)
    
    cv2.imshow('vot ono politeh', cv_image)
    cv2.waitKey(1)

# rospy.wait_for_message('')
rospy.sleep(8)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(10)
navigate(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
rospy.sleep(1)

navigate(x=4, y=3, z=1, speed=1, frame_id='aruco_map')
rospy.sleep(8)

land()
rospy.sleep(3)
out.release()
cv2.destroyAllWindows()

rospy.spin()
