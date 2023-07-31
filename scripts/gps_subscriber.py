#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix


def gps_callback(data):
    # Print GPS data received from /gps topic
    rospy.loginfo("Received GPS Data:")
    rospy.loginfo("Time: {}".format(data.header.stamp))
    rospy.loginfo("Latitude: {}".format(data.latitude))
    rospy.loginfo("Longitude: {}".format(data.longitude))
    rospy.loginfo("Altitude: {}".format(data.altitude))
    rospy.loginfo("---------------------------")

def latitude_callback(data):
    # Print latitude data received from /gps/latitude topic
    rospy.loginfo("Received Latitude Data: {}".format(data.data))

def longitude_callback(data):
    # Print longitude data received from /gps/longitude topic
    rospy.loginfo("Received Longitude Data: {}".format(data.data))

def altitude_callback(data):
    # Print altitude data received from /gps/altitude topic
    rospy.loginfo("Received Altitude Data: {}".format(data.data))

def timedata_callback(data):
    # Print time data received from /gps/timedata topic
    rospy.loginfo("Received Time Data: {}".format(data.data))

def gps_subscriber():
    rospy.init_node('gps_subscriber_node', anonymous=True)

    # Subscribe to the topics
    rospy.Subscriber('/gps', NavSatFix, gps_callback)
    rospy.Subscriber('/gps/latitude', Float32, latitude_callback)
    rospy.Subscriber('/gps/longitude', Float32, longitude_callback)
    rospy.Subscriber('/gps/altitude', Float32, altitude_callback)
    rospy.Subscriber('/gps/timedata', String, timedata_callback)

    # Spin until interrupted
    rospy.spin()

if __name__ == "__main__":
    try:
        gps_subscriber()
    except rospy.ROSInterruptException:
        pass