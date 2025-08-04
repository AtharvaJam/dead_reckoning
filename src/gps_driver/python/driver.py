#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from gps_driver.msg import gps_msg
from std_msgs.msg import Time
import utm
import sys


if __name__ == '__main__':
    SENSOR_NAME = "gps"
    rospy.init_node('gps_driver')
    # print(sys.argv)
    portname = sys.argv[1]
    try:
        serial_port = rospy.get_param('~port',portname)
        serial_baud = rospy.get_param('~baudrate', 4800)
        port = serial.Serial(serial_port, serial_baud, timeout=3.)
    except serial.serialutil.SerialException: 
        rospy.loginfo("Serial Port not found, shutting down.")
        rospy.signal_shutdown("Port not found")


    rospy.logdebug("Using gnss sensor on port "+serial_port+" at "+str(serial_baud))
    gnss_pub = rospy.Publisher('gps', gps_msg, queue_size=5)
    
    rospy.logdebug("Initialization complete")
    
    rospy.loginfo("Publishing GNSS data.")
    

    gnss_msg = gps_msg()
    gnss_msg.header.frame_id = "GPS1_Frame"

    gnss_msg.header.seq=0
    timestamp = Time()
    gnss_msg.header.stamp.nsecs = 0
    
    try:
        while not rospy.is_shutdown():
            try:
                newdata =  port.readline()[2:].decode("utf-8").strip()
                # newdata =port.readline()[1:].strip()
                # print(newdata)
                if newdata == '':
                    rospy.logwarn("No data")
                else:
                    if newdata.startswith("GPGGA") or newdata.startswith("PGGA"): 
                        # print("In gpgga")
                        
                        gpgga = newdata.split(',')
                        timestamp= float(gpgga[1])
                        # gnss_msg.header.stamp.seq=gnss_msg.header.seq
                        gnss_msg.header.stamp.secs = int(timestamp)
                        gnss_msg.header.stamp.nsecs = int((timestamp%1)*(10**9))
                        if(gpgga[2]!=''):

                            lat = float(gpgga[2])
                            lon = float(gpgga[4])
                            alt = float(gpgga[9])
                            # print(lat, lon, alt)
                            # print(lat, lon)

                            lat1 = int(lat/100) + ((lat%100)/60)
                            lon1 = int(lon/100) + ((lon%100)/60)
                            if gpgga[3]=='S':
                                lat1*=-1
                            if gpgga[5]=='W':
                                lon1*=-1
                            utm_data = utm.from_latlon(lat1, lon1)
                            gnss_msg.latitude = lat1
                            gnss_msg.longitude = lon1
                            gnss_msg.altitude = alt
                            gnss_msg.utm_easting = float(utm_data[0])
                            gnss_msg.utm_northing = float(utm_data[1])
                            gnss_msg.zone =int(utm_data[2])
                            gnss_msg.letter = utm_data[3]
                            # print(gpgga[1])
                            # print(gnss_msg)
                        else:
                            rospy.loginfo("No data stream, Go outside")
                        gnss_pub.publish(gnss_msg)
                        gnss_msg.header.seq+=1
            except UnicodeDecodeError:
                rospy.loginfo("Incoherent data read, skipping this timestamp data.")
                continue

                        
            
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down gnss_driver node...")
