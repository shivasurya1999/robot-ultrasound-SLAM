#!/usr/bin/env python3
import rospy
import tf2_ros
import tf_conversions
import yaml
import geometry_msgs.msg
from tf.transformations import quaternion_multiply, translation_matrix, quaternion_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix, inverse_matrix

def read_cam_to_ee_transform(yaml_file):
    with open(yaml_file, 'r') as file:
        cam_to_ee_data = yaml.safe_load(file)["cam_to_ee"]
    return cam_to_ee_data

# def transform_to_matrix(transform):
#     return concatenate_matrices(translation_matrix([transform['translation']['x'], transform['translation']['y'], transform['translation']['z']]),
#                                 quaternion_matrix([transform['quaternion']['x'], transform['quaternion']['y'], transform['quaternion']['z'], transform['quaternion']['w']]))

def transform_to_matrix(transform):
    if isinstance(transform, dict):
        # Assuming the structure of your dictionary matches the YAML format mentioned
        translation = [transform['translation']['x'], transform['translation']['y'], transform['translation']['z']]
        rotation = [transform['quaternion']['x'], transform['quaternion']['y'], transform['quaternion']['z'], transform['quaternion']['w']]
    else:
        # Handling Transform message object
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
    
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(rotation))



def matrix_to_transform(matrix):
    trans = translation_from_matrix(matrix)
    quat = quaternion_from_matrix(matrix)
    return trans, quat

def listen_tf(tf_buffer, from_frame, to_frame):
    try:
        trans_stamped = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        return trans_stamped.transform, trans_stamped.header.stamp
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None, None


def write_ground_truth(timestamp, transform, file_path):
    secs = timestamp.secs + timestamp.nsecs / 1e9  # Convert to floating-point seconds
    with open(file_path, 'a') as file:
        file.write(f"{secs} {transform[0][0]} {transform[0][1]} {transform[0][2]} {transform[1][0]} {transform[1][1]} {transform[1][2]} {transform[1][3]}\n")


# if __name__ == '__main__':
#     rospy.init_node('tf_listener')

#     tf_buffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tf_buffer)

#     cam_to_ee_data = read_cam_to_ee_transform(f'/bagfiles/cam_to_link7_quat.yaml')
#     cam_to_ee_matrix = transform_to_matrix(cam_to_ee_data)

#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
#         ground_to_link7_transform, timestamp = listen_tf(tf_buffer, 'panda_link0', 'panda_link7')
#         if ground_to_link7_transform is not None and timestamp is not None:
#             ground_to_link7_matrix = transform_to_matrix(ground_to_link7_transform)
#             cam_to_ground_matrix = concatenate_matrices(ground_to_link7_matrix, cam_to_ee_matrix)
#             cam_to_ground_transform = matrix_to_transform(cam_to_ground_matrix)
#             write_ground_truth(timestamp, cam_to_ground_transform, f'/bagfiles/groundtruth.txt')
#         rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Read the transformation from the camera to the end-effector (link7) from the YAML file
    # cam_to_ee_data = read_cam_to_ee_transform('/bagfiles/cam_to_link7_quat.yaml')
    # cam_to_ee_data = read_cam_to_ee_transform('/fusionlab_files/data/bag1/cam_to_link7_quat.yaml')
    cam_to_ee_data = read_cam_to_ee_transform('/fusionlab_files/data/bag15/cam_to_link7_quat.yaml')
    # Convert the transformation data to a 4x4 matrix
    cam_to_ee_matrix = transform_to_matrix(cam_to_ee_data)
    # # Invert the transformation to get from link7 to the camera
    # link7_to_cam_matrix = inverse_matrix(cam_to_ee_matrix)

    rate = rospy.Rate(10.0)

    # while not rospy.is_shutdown():
    #     # Listen for the transformation from the ground (panda_link0) to link7 (panda_link7)
    #     # ground_to_link7_transform, timestamp = listen_tf(tf_buffer, 'panda_link0', 'panda_link7')
    #     ground_to_link7_transform, timestamp = listen_tf(tf_buffer, 'fr3_link0', 'fr3_link7')
    #     if ground_to_link7_transform is not None and timestamp is not None:
    #         # Convert the transformation to a 4x4 matrix
    #         ground_to_link7_matrix = transform_to_matrix(ground_to_link7_transform)
    #         # Concatenate the transformations to compute the transformation from the ground to the camera
    #         ground_to_cam_matrix = concatenate_matrices(ground_to_link7_matrix, link7_to_cam_matrix)
    #         # Convert the resulting transformation matrix back to translation and rotation components
    #         ground_to_cam_transform = matrix_to_transform(ground_to_cam_matrix)
    #         # Write the transformation data to the groundtruth.txt file, including the timestamp
    #         # write_ground_truth(timestamp, ground_to_cam_transform, '/bagfiles/groundtruth.txt')
    #         # write_ground_truth(timestamp, ground_to_cam_transform, '/fusionlab_files/data/bag1/groundtruth.txt')
    #         write_ground_truth(timestamp, ground_to_cam_transform, '/fusionlab_files/data/bag12/groundtruth.txt')
    #     rate.sleep()

    while not rospy.is_shutdown():
        # Listen for the transformation from the ground (panda_link0) to link7 (panda_link7)
        # ground_to_link7_transform, timestamp = listen_tf(tf_buffer, 'panda_link0', 'panda_link7')
        link7_to_ground_transform, timestamp = listen_tf(tf_buffer, 'fr3_link7', 'fr3_link0')
        if link7_to_ground_transform is not None and timestamp is not None:
            # Convert the transformation to a 4x4 matrix
            link7_to_ground_matrix = transform_to_matrix(link7_to_ground_transform)
            # Concatenate the transformations to compute the transformation from the ground to the camera
            cam_to_ground_matrix = concatenate_matrices(link7_to_ground_matrix, cam_to_ee_matrix)
            # Convert the resulting transformation matrix back to translation and rotation components
            ground_to_cam_transform = matrix_to_transform(cam_to_ground_matrix)
            # Write the transformation data to the groundtruth.txt file, including the timestamp
            # write_ground_truth(timestamp, ground_to_cam_transform, '/bagfiles/groundtruth.txt')
            # write_ground_truth(timestamp, ground_to_cam_transform, '/fusionlab_files/data/bag1/groundtruth.txt')
            write_ground_truth(timestamp, ground_to_cam_transform, '/fusionlab_files/data/bag15/groundtruth_new.txt')
        rate.sleep()