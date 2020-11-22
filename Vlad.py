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
    cv2.putText(cv_image, 'y = '+str("%.2f" % telem.y), (0,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
    cv2.putText(cv_image, 'x = '+str("%.2f" % telem.x), (0,24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
    cv2.putText(cv_image, 'z = '+str("%.2f" % telem.z), (0,36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
    out.write(cv_image)
    
    cv2.imshow('nti', cv_image)
    cv2.waitKey(1)

# rospy.wait_for_message('')
rospy.sleep(8)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)


land()
rospy.sleep(3)
out.release()
cv2.destroyAllWindows()

rospy.spin()
