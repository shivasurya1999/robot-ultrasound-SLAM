#!/usr/bin/env python3

def invert_quaternion(quaternion):
    """
    Invert a quaternion to represent the opposite rotation.
    Quaternion inversion is done by negating the x, y, z components and keeping w unchanged.
    """
    x, y, z, w = quaternion
    return [-x, -y, -z, w]

def apply_quaternion_to_vector(quaternion, vector):
    """
    Rotate a vector by a quaternion.
    """
    # Quaternion multiplication (q * v * q^-1)
    q = quaternion
    v = [0] + vector  # Extend vector to quaternion with 0 as scalar part
    q_conj = invert_quaternion(quaternion)
    
    # First multiplication (q * v)
    qv = quaternion_multiply(q, v)
    # Second multiplication ((q * v) * q^-1)
    v_rotated = quaternion_multiply(qv, q_conj)
    
    # Return the vector part of the result
    return v_rotated[1:]

def invert_transformation(translation, quaternion):
    """
    Invert a transformation composed of a translation and a quaternion rotation.
    """
    # Invert the quaternion (rotation)
    inv_quaternion = invert_quaternion(quaternion)
    # Apply the inverted quaternion to the translation vector and then negate it
    inv_translation = apply_quaternion_to_vector(inv_quaternion, translation)
    inv_translation = [-x for x in inv_translation]
    
    return inv_translation, inv_quaternion

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    return [w, x, y, z]

def process_file(input_filepath, output_filepath):
    try:
        with open(input_filepath, 'r') as input_file, open(output_filepath, 'w') as output_file:
            for line in input_file:
                parts = line.strip().split()
                timestamp = parts[0]
                translation = [float(parts[1]), float(parts[2]), float(parts[3])]
                quaternion = [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])]
                
                inverted_translation, inverted_quaternion = invert_transformation(translation, quaternion)
                
                # Write to output file
                output_file.write(f"{timestamp} {' '.join(map(str, inverted_translation))} {' '.join(map(str, inverted_quaternion))}\n")
    except FileNotFoundError as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    input_filepath = '/home/ssuryalolla/fusionlab_files/data/bag10/groundtruth.txt'  # Adjust this path
    output_filepath = '/home/ssuryalolla/fusionlab_files/data/bag10/inverted_groundtruth.txt'  # Adjust this path
    process_file(input_filepath, output_filepath)
