# read world file and double the x,y,z values of pose frame using regular expression

import re
import os
import math

# define input and output file paths
working_dir = "/media/j/T7/git/hanwha/src/natural_enviroments/worlds/"
input_file = "lake_0.world"
output_file = "lake_1.world"

# define transformation parameters
scale = (1, 1, 1)  # Scale factors for x, y, z
yaw_rotation = math.radians(0)  # Rotation angle around the z-axis in radians
# translation = (0, -0, 0)  # Translation vector (x, y, z)
translation = (50, -50, 0)  # Translation vector (x, y, z)

if os.path.isfile(output_file):
    os.remove(output_file)

# define the pattern to match
# pattern_pose = re.compile(r"(\s+)<pose frame=''>([a-zA-Z0-9-.]+)\s+([a-zA-Z0-9-.]+)\s+([a-zA-Z0-9-.]+)\s+([a-zA-Z0-9-.]+)\s+([a-zA-Z0-9-.]+)\s+([a-zA-Z0-9-.]+)</pose>")
pattern_pose = re.compile(r"(\s+)<pose frame=''>([0-9eE+.-]+)\s+([0-9eE+.-]+)\s+([0-9eE+.-]+)\s+([0-9eE+.-]+)\s+([0-9eE+.-]+)\s+([0-9eE+.-]+)</pose>")

def rotate_z(x, y, r, yaw_rotation):
    # Apply rotation around z-axis (yaw)
    cosc = math.cos(yaw_rotation)
    sinc = math.sin(yaw_rotation)
    x_new = x * cosc - y * sinc
    y_new = x * sinc + y * cosc
    r_new = r + yaw_rotation
    return x_new, y_new, r_new

def modify_pose(match, scale, yaw_rotation, translation):
    indent, x, y, z, p, q, r = match.groups()
    x, y, z, r = map(float, (x, y, z, r))

    # Apply scale
    x = x * scale[0]
    y = y * scale[1]
    z = z * scale[2]

    # Apply z-axis rotation
    x, y, r = rotate_z(x, y, r, yaw_rotation)

    # Apply translation
    x += translation[0]
    y += translation[1]
    z += translation[2]

    modifed_line = indent + f"<pose frame=''>{x} {y} {z} {p} {q} {r}</pose>" + "\n" 
    return modifed_line

with open(working_dir + input_file, 'r') as input_file, open(working_dir + output_file, 'w') as output_file:
    for line in input_file:
        match_pose = pattern_pose.match(line)
        if match_pose:
            output_file.write(modify_pose(match_pose, scale, yaw_rotation, translation))
        else:
            output_file.write(line)
