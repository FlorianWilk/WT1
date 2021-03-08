#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import os
import sys

msg=""

def callback(data):
    global msg
    #rospy.loginfo("got data {}",data)
    msg=b"s"+str(int(data.linear.x))+","+str(int(data.linear.y))+"\n"

def talker():
    global msg
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    rospy.logerr("doing fancy shit *************")
    pub = rospy.Publisher('/wt1/serial', String, queue_size=10)
    rospy.Subscriber("/wt1/vel", Twist, callback)
    #rospy.sleep(10)
    isrunning=True
    instr=""
    while isrunning:
        try:
            rospy.logerr("Trying to connect to Serial")
            ser = serial.Serial('/dev/ttyACM0',115200)  # open serial port
            rospy.sleep(1)
            rospy.logerr("starting")
            while not rospy.is_shutdown() and ser.is_open:
                if msg is "":
                  ser.write(b"#")     # write a string
                else:
                    #rospy.loginfo("Writing messgae {}",msg)
                    ser.write(msg)
                msg=""
                newstr=""
                while ser.in_waiting:
                    newstr+=ser.read()             
                rate.sleep()
                rospy.logerr(newstr)
                pub.publish(String(newstr))
                instr+=newstr
            isrunning=False
            ser.close()
        except serial.SerialException:
            rospy.logerr(sys.exc_info())
        rospy.sleep(2)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
