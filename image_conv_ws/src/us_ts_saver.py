#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import os

def callback(data):
    # Ensure the directory exists
    # base_path = '/fusionlab_files/data/bag1'
    base_path = '/fusionlab_files/data/bag15'
    file_path = os.path.join(base_path, 'us_img_timestamps.txt')
    os.makedirs(base_path, exist_ok=True)

    # Convert rospy.Time to floating-point seconds
    secs = data.header.stamp.secs + data.header.stamp.nsecs / 1e9
    with open(file_path, 'a') as file:
        file.write(f"{secs}\n")
        # Optional: print a message indicating the save
        rospy.loginfo(f"US Timestamp saved: {secs}")

def listener():
    rospy.init_node('us_ts_saver', anonymous=True)
    # rospy.Subscriber("/camera/depth/image", Image, callback)
    rospy.Subscriber("/ultrasound", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
