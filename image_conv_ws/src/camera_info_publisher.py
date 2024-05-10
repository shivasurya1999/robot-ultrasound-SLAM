#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo

def publish_camera_info():
    rospy.init_node('camera_info_publisher', anonymous=True)
    pub = rospy.Publisher('/cam2/color/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        camera_info = CameraInfo()
        
        camera_info.header.frame_id = "cam2_color_optical_frame"
        camera_info.width = 1280  # Adjust accordingly
        camera_info.height = 720  # Adjust accordingly
        
        # Set camera intrinsic parameters
        camera_info.K = [636.309, 0.0, 639.272,  # fx, 0, cx
                         0.0, 636.309, 360.595,  # 0, fy, cy
                         0.0, 0.0, 1.0]          # 0, 0, 1
        camera_info.P = [636.309, 0.0, 639.272, 0.0,  # fx, 0, cx, 0
                         0.0, 636.309, 360.595, 0.0,  # 0, fy, cy, 0
                         0.0, 0.0, 1.0, 0.0]          # 0, 0, 1, 0
        
        # Assuming no distortion
        camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish the CameraInfo message
        pub.publish(camera_info)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
