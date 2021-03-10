#! /usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray


def callback(data):
    global c
    ultra_msg = data.data
    c.center = ultar_msg[2]
    
    if ultra_msg[1] > 200:
        c.left = 200
    else:
        c.left = ultra_msg[1]
    
    if ultra_msg[3] > 200:
        c.right = 200
    else:
        c.right = ultra_msg[3]
        
    if ultra_msg[0] > 150:
        c.r_left = 150
    else:
        c.r_left = ultra_msg[0]
    
    if ultra_msg[4] > 150:
        c.r_right = 150
    else:
        c.r_right = ultra_msg[4]
        
    return c.left, c.right, c.r_left, c.r_right, c.center


class CALCULATE:

    def __init__(self):
        self.ultra_msg = 0
        self.left = 0
        self.center = 0
        self.right = 0
        self.r_left = 0
        self.r_right = 0
        self.angle = 0
        self.speed = 0
        
    def drive_go(self):
        self.speed = 50
        self.angle = ((-self.left - 15 + self.right + 15)*0.8 - self.r_left * 1.5 + self.r_right * 1.5)
        
        if self.center <= 100:
            self.speed = 45
            self.angle = ((-self.left - 15 + self.right + 15)*0.8 - self.r_left * 1.5 + self.r_right * 1.5)
        
        return self.angle, self.speed
        
    def drive_stop(self):
        for i in range(5):
            self.speed = 0
            self.angle = 0
            time.sleep(0.1)
            
        for i in range(7):
            self.angle = 0
            self.speed = -30
            time.sleep(0.15)
	
	return self.angle, self.speed

c = CALCULATE()
      
def main():
    if c.center < 30:
        c.drive_stop()
    else:
        c.drive_go()
    c.drive_go()
    print("ss")
    return c.angle, c.speed 
        
if __name__ == '__main__':
    rospy.init_node('ultrasonic')
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, callback)
    main()
