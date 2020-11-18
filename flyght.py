from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray

import math
from pyzbar.pyzbar import decode
from PIL import Image as pimg
# ------------------------------------------------------------------------------------------------------------------------------

rospy.init_node('flight')
bridge = CvBridge()
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (320, 240))

def image_call(data):
    telem = get_telemetry(frame_id='aruco_map')
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv2.putText(cv_image, 'x = '+str("%.2f" % telem.x), (0,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
    cv2.putText(cv_image, 'y = '+str("%.2f" % telem.y), (0,27), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
    cv2.putText(cv_image, 'z = '+str("%.2f" % telem.z), (0,42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
    out.write(cv_image)
    cv2.imshow('politeh', cv_image)
    cv2.waitKey(1)


def qr_read():

    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cv2.imwrite('qr.jpg', img)
    barcode = decode(pimg.open('qr.jpg'))
    print(barcode[0].data)
# ------------------------------------------------------------------------------------------------------------------------------


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)




coords = {1: [0, 0, 1],
          3: [2, 2, 1],
          5: [0, 8, 1],
          7: [2, 8, 1],
          6: [8, 5, 1],
          4: [10, 0, 1],
          2: [10, 10, 1]}

path = [3,4,5]

sensor_value = []
# ------------------------------------------------------------------------------------------------------------------------------

image_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_call)
navigate(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(2)
navigate(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
rospy.sleep(1)
for i in path:
    print("go to ", i, coords[i])
    navigate(x = coords[i][0], y= coords[i][1], z = coords[i][2], speed = 2.5, frame_id = 'aruco_map')
    if i==4:
        rospy.sleep(8)
        print("read qr_code...")
        navigate(x = coords[i][0], y= coords[i][1], z = coords[i][2]-0.5, speed = 0.5, frame_id = 'aruco_map')
        rospy.sleep(5)
        try:
            qr_read()
        except:
            print('not read qr')
        continue
    rospy.sleep(8)
    try:
        s = rospy.wait_for_message('sensors',Float32MultiArray).data[0]
        sensor_value.append(s)
    except:
        print('--')
navigate(x=0, y=0, z=1, speed=2, frame_id='aruco_map')
rospy.sleep(7)
land()
rospy.sleep(2)

print(sensor_value)
out.release()
cv2.destroyAllWindows()
