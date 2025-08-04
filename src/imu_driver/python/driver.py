#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from imu_driver.msg import imu_msg
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Time
import sys
import numpy as np

# Input Euler Angles in Radians
def euler_to_quaternion(yaw, pitch, roll):
    q=Quaternion()
    q.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return q


if __name__ == '__main__':
    SENSOR_NAME = "imu"
    rospy.init_node('imu_driver')
    # print(sys.argv)
    # rospy.sleep(5)
    portname = sys.argv[1]
    try:
        serial_port = rospy.get_param('~port',portname)
        serial_baud = rospy.get_param('~baudrate', 115200)
        port = serial.Serial(serial_port, serial_baud, timeout=3)
        port.write(b"$VNWRG,07,40*xx") #Change frequency to 40Hz


    except serial.serialutil.SerialException:

        rospy.loginfo("Serial Port not found, shutting down.")
        rospy.signal_shutdown("Port not found")


    rospy.logdebug("Using imu sensor on port "+serial_port+" at "+str(serial_baud))
    gnss_pub = rospy.Publisher('imu', imu_msg, queue_size=5)
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing IMU data.")
    

    imu_msg = imu_msg()
    imu_msg.Header.frame_id = "IMU1_Frame"
    imu_msg.Header.seq=0
    
    try:
        while not rospy.is_shutdown():
            try:
            
                newdat =  port.readline()[:-5].decode("utf-8")
                imu_msg.data = newdat
                newdata = newdat.strip()
                # print(newdata)
                if newdata == '':
                    rospy.logwarn("No data")
                else:
                    if "$VNYMR" in newdata: 
                        # print("In vnymr")
                        
                        vnymr = newdata.split(',')
                        # print(vnymr)
                        if(vnymr[2]!=''):
                            timestamp= float(vnymr[1])
                            # imu_msg.header.stamp.seq=imu_msg.header.seq
                            imu_msg.Header.stamp = rospy.Time.now()
                            # print(vnymr)

                            # Orientation
                            yaw = np.radians(float(vnymr[1]))
                            pitch = np.radians(float(vnymr[2]))
                            roll = np.radians(float(vnymr[3]))
                            imu_msg.IMU.orientation = euler_to_quaternion(yaw, pitch, roll)

                            # Angular Velocity
                            ang_vel = Vector3()
                            ang_vel.x = float(vnymr[10])
                            ang_vel.y = float(vnymr[11])
                            ang_vel.z = float(vnymr[12])
                            imu_msg.IMU.angular_velocity = ang_vel

                            # Linear Acceleration 
                            lin_accl = Vector3()
                            lin_accl.x = float(vnymr[7])
                            lin_accl.y = float(vnymr[8])
                            lin_accl.z = float(vnymr[9])      
                            imu_msg.IMU.linear_acceleration = lin_accl

                            # imu_msg.IMU.header = imu_msg.Header

                            # Magnetic Field
                            mag_fld = Vector3()
                            mag_fld.x = float(vnymr[4])
                            mag_fld.y = float(vnymr[5])
                            mag_fld.z = float(vnymr[6])   
                            imu_msg.MagField.magnetic_field = mag_fld  
                            # imu_msg.MagField.header = imu_msg.Header                             

                        else:
                            rospy.loginfo("No data stream, Go outside")
                            continue
                        gnss_pub.publish(imu_msg)
                        imu_msg.Header.seq+=1
            except UnicodeDecodeError:
                rospy.loginfo("Incoherent data read, skipping this timestamp data.")
                continue

                        
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu_driver node...")