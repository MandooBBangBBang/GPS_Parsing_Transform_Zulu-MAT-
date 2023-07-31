#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta
import re
import serial

# Parse GGA and RMC sentences and extract required data using regular expressions
def parse_gps_data(gps_data):
    gga_pattern = r"\$G[PN]GGA,(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d+\.\d+),\d+,\d+,\d+\.\d+,(\d+\.\d+),M"
    rmc_pattern = r"\$G[PN]RMC,(\d+\.\d+),[AV],(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d+\.\d+),,\d+,,(\d{2})(\d{2})(\d{2}),[D|S]"

    gga_match = re.search(gga_pattern, gps_data)
    rmc_match = re.search(rmc_pattern, gps_data)

    if gga_match:
        gga_time = gga_match.group(1)
        latitude = float(gga_match.group(2)) * (-1 if gga_match.group(3) == 'S' else 1)
        longitude = float(gga_match.group(4)) * (-1 if gga_match.group(5) == 'W' else 1)
        altitude = float(gga_match.group(6))
    else:
        gga_time, latitude, longitude, altitude = None, None, None, None

    if rmc_match:
        rmc_date = rmc_match.group(7) + rmc_match.group(8) + rmc_match.group(9)
    else:
        rmc_date = None

    return gga_time, latitude, longitude, altitude, rmc_date

# Publish GPS data to ROS topics
def publish_gps_data(gps_data, publisher):
    gga_time, latitude, longitude, altitude, rmc_date = parse_gps_data(gps_data)

    if gga_time is not None and latitude is not None and longitude is not None and altitude is not None and rmc_date is not None:
        # Convert UTC time to Seoul time (UTC+9)
        utc_time = datetime.strptime(gga_time, "%H%M%S.%f")
        seoul_time = utc_time + timedelta(hours=9)
        time_data = seoul_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # Print GPS data in the specified format
        rospy.loginfo("Time : {}".format(time_data))
        rospy.loginfo("Latitude : {}".format(latitude))
        rospy.loginfo("Longitude : {}".format(longitude))
        rospy.loginfo("Altitude : {}".format(altitude))
        rospy.loginfo("---------------------------")

        # Create messages and publish to ROS topics
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.stamp = rospy.Time.now()
        navsatfix_msg.latitude = latitude
        navsatfix_msg.longitude = longitude
        navsatfix_msg.altitude = altitude

        time_msg = String()
        time_msg.data = time_data

        # Publish messages to respective topics
        publisher['/gps'].publish(navsatfix_msg)
        publisher['/gps/latitude'].publish(Float32(latitude))
        publisher['/gps/longitude'].publish(Float32(longitude))
        publisher['/gps/altitude'].publish(Float32(altitude))
        publisher['/gps/timedata'].publish(time_msg)

# Main function
if __name__ == "__main__":
    rospy.init_node('gps_parser', anonymous=True)

    # Create publishers for ROS topics
    gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=10)
    lat_pub = rospy.Publisher('/gps/latitude', Float32, queue_size=10)
    lon_pub = rospy.Publisher('/gps/longitude', Float32, queue_size=10)
    alt_pub = rospy.Publisher('/gps/altitude', Float32, queue_size=10)
    time_pub = rospy.Publisher('/gps/timedata', String, queue_size=10)

    publisher = {
        '/gps': gps_pub,
        '/gps/latitude': lat_pub,
        '/gps/longitude': lon_pub,
        '/gps/altitude': alt_pub,
        '/gps/timedata': time_pub
    }

    # Initialize serial communication
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

    while not rospy.is_shutdown():
        # Read GPS data from the serial port
        gps_data = ser.readline().decode('ascii', errors='ignore').strip()

        # Check if the GPS data is valid and not empty
        if gps_data.startswith('$G') and len(gps_data) > 0:
            publish_gps_data(gps_data, publisher)
        else:
            rospy.logwarn("Invalid GPS data received.")

    ser.close()