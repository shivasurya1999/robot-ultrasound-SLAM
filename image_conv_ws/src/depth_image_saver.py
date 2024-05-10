#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Global sequence number for image filenames
image_seq = 0

def callback(data):
    global image_seq  # Use the global sequence number
    bridge = CvBridge()
    try:
        # Convert the depth image using the "passthrough" encoding
        cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    except Exception as e:
        print(e)
    else:
        # Save your image with an incrementing filename
        # filename = f'/bagfiles/bagfile2/depth/frame{image_seq:04}.png'
        # filename = f'/fusionlab_files/data/bag1/depth/frame{image_seq:04}.png'
        filename = f'/fusionlab_files/data/bag15/depth/frame{image_seq:04}.png'
        cv2.imwrite(filename, cv_image)
        print(f"Saved {filename}")  # Optional: print a message indicating the save
        image_seq += 1  # Increment the sequence number for the next image

def listener():
    rospy.init_node('depth_image_saver', anonymous=True)
    # rospy.Subscriber("/camera/depth/image", Image, callback)
    rospy.Subscriber("/cam2/depth/image", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
