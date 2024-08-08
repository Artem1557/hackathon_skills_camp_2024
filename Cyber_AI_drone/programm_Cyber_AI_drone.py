import threading
import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


import math
from clover.srv import SetLEDEffect

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
image_pub = rospy.Publisher('/CyberAI_drone_debug', Image, queue_size=1)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

set_effect(r=0, g=255, b=0)  
navigate_wait(z=1, frame_id='body', auto_arm=True)
set_effect(r=255, g=255, b=0)
navigate_wait(x=2, y=1.5, z=1, frame_id='aruco_map')

led_effect_running = False

def led_effect():
    global led_effect_running
    while led_effect_running:
        set_effect(effect='blink', r=255, g=0, b=0)
        rospy.sleep(0.2)
        set_effect(effect='blink', r=0, g=0, b=255)
        rospy.sleep(0.2)

num =0
def home(num):
    if (num == 2):
        rospy.sleep(5)
        navigate_wait(x = 0, y = 0, z=1, frame_id='aruco_map')
        rospy.sleep(3)
        set_effect(r=204, g=255, b=0)
        rospy.sleep(3)
        set_effect(r=204, g=255, b=0)
        rospy.sleep(3)
        land()
        set_effect(r=204, g=255, b=0)
        rospy.sleep(3)


while True:
    global img
    img = CvBridge().imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_rover = cv2.inRange(hsv, (70,150,150,), (80,255,255))
    mask_red1 = cv2.inRange(hsv, (0, 110, 150), (7, 255, 255))
    mask_red2 = cv2.inRange(hsv, (172, 110, 150), (180, 255, 255))
    mask_green = cv2.inRange(hsv, (60, 150, 150), (70, 255, 255))
    mask_line = cv2.inRange(hsv, (0, 0, 183), (0, 0, 200))

    Mrov = cv2.moments(mask_rover)
    Mred1 = cv2.moments(mask_red1)
    Mred2 = cv2.moments(mask_red2)
    Mgreen = cv2.moments(mask_green)
    Mline = cv2.moments(mask_line)
    

    if Mrov["m00"] >0:
        cX = int(Mrov["m10"]/Mrov["m00"])
        cY = int(Mrov["m01"]/Mrov["m00"])
        cv2.putText(img, 'rover', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 160,0), 1)
        set_effect(r=220, g=20, b=60)
        rospy.sleep(0.5)

    if Mred1["m00"] >0:
        cX = int(Mred1["m10"]/Mred1["m00"])
        cY = int(Mred1["m01"]/Mred1["m00"])
        cv2.putText(img, 'rover', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0, 255), 1)
        set_effect(r=255, g=165, b=0)
        rospy.sleep(0.35)


    if Mgreen["m00"] >0:
        cX = int(Mgreen["m10"]/Mgreen["m00"])
        cY = int(Mgreen["m01"]/Mgreen["m00"])
        cv2.putText(img, 'green', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,255,0), 1)
        set_effect(r=255, g=165, b=0)
        rospy.sleep(0.35)

    if Mline["m00"] >0:
        cX = int(Mline["m10"]/Mline["m00"])
        cY = int(Mline["m01"]/Mline["m00"])
        cv2.putText(img, 'line', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (170, 200,75), 1)
        set_effect(r=255, g=192, b=203)
        rospy.sleep(0.35)

    image_pub.publish(CvBridge().cv2_to_imgmsg(img, 'bgr8'))

    if Mred1["m00"] >0 and Mrov["m00"] >0:
        cX1 = int(Mrov["m10"]/Mrov["m00"])
        cY = int(Mrov["m01"]/Mrov["m00"])
        cX2 = int(Mline["m10"]/Mline["m00"])
        if cX1>=cX2:
            print(u'прижмитесь к обочине'.encode('utf-8').decode('utf-8'))
            led_effect_running = True
            led_effect_thread = threading.Thread(target=led_effect)
            led_effect_thread.start()
            for i in range(2):
                i+1
                num +=1
        
                if Mrov["m00"] >0:
                    cX = int(Mrov["m10"]/Mrov["m00"])
                    cY = int(Mrov["m01"]/Mrov["m00"])
                    cv2.putText(img, 'stop', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 160,0), 1)
                    
                    if (cY == int(Mrov["m01"]/Mrov["m00"])):
                        navigate_wait(x = 2 , y = 0 , frame_id="body", speed=0.6)
                        
                    if (cX1 == int(Mrov["m10"]/Mrov["m00"])):
                        navigate_wait(x = 0 , y = 1 , frame_id="body", speed=0.6)
                        

                    if (cX <= int(Mrov["m10"]/Mrov["m00"])):
                        navigate_wait(x = -3 , y = 0 , frame_id="body", speed=0.6)
                        

                    if (cY <= int(Mrov["m01"]/Mrov["m00"])):
                        navigate_wait(x = 0 , y = -1 , frame_id="body", speed=0.6)

                rospy.sleep(2)
            led_effect_running = False
            led_effect_thread.join()  
        home(num)