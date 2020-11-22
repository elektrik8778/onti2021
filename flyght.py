#import  bibliotec
from __future__ import print_function
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray

import math
# from pyzbar.pyzbar import decode
from PIL import Image as pimg
#-----------------------------------------------
def read_sensor():
    data = rospy.wait_for_message('/sensors', Float64MultiArray).data
    print(data)
    return data
    
def qr_read():
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cv2.imwrite('qr.jpg', img)
    barcode = decode(pimg.open('qr.jpg'))
    print(barcode[0].data.encode("utf-8"))

def image_callback(data):
    telem = get_telemetry(frame_id='aruco_map')
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    cv2.putText(cv_image, 'y = '+str("%.1f" % telem.y), (0,12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)
    cv2.putText(cv_image, 'x = '+str("%.1f" % telem.x), (0,24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)
    cv2.putText(cv_image, 'z = '+str("%.1f" % telem.z), (0,36), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
    out.write(cv_image)

    cv2.imshow('vot ono politeh', cv_image)
    cv2.waitKey(1)


rospy.init_node('flight')
bridge = CvBridge()
out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 10, (320, 240))

# ------------------------------------------------------------------------------------------------------------------------------


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)

land = rospy.ServiceProxy('land', Trigger)




coords =  {1: [8.0 ,0.95 , 2],
          8: [5.40 ,0.03,2 ],
          3: [3.64 ,6.25 , 2],
          5: [1.02 ,8.07 , 2],
          10: [6.24 ,7.04,2 ],
          7: [3.29 ,3.06 , 2],
          6: [8.27 ,4.73 , 2],
          4: [1.06 ,0.97 , 2],
          9: [0.84 ,4.93 ,2 ],
          2: [8.98 ,8.88 , 2]}

path = [1, 2, 3, 4, 5, 6, 7 ,8 ,9 , 10]

zamer = [3,5,7,8]

# ------------------------------------------------------------------------------------------------------------------------------

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)

#polyot 
print ('vzlet')

navigate(x=0, y=0, z=2, speed=1, frame_id='body', auto_arm=True)
rospy.sleep(5)
navigate(x=0, y=0, z=2, speed=0.5, frame_id='aruco_map')
rospy.sleep(1)

# qr_read()

# print('not read qr(')
rospy.sleep(2)
print('uletau na zadanie!')
rospy.sleep(2)

for i in path:
    print("go to ", i, coords[i])
    
    navigate(x = coords[i][0], y= coords[i][1], z = 2, speed = 1, frame_id = 'aruco_map')
    rospy.sleep(12)
    
    if i in zamer:
        print('zamer...')        
        
        with open('dannie_POLITEH.txt', 'a') as f:
            f.write('v tochke '+str(i)+' '+str(coords[i])+' '+ str(read_sensor())+'\n')






#go to home
print('go to home')
navigate(x=0, y=0, z=2, speed=1.5, frame_id='aruco_map')
rospy.sleep(12)
land()


out.release()
cv2.destroyAllWindows()

