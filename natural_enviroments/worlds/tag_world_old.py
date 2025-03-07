# read world file and replace the strings in the list with the corresponding strings in the dictionary

import os
import tag_list_old as tag_list

tagdic = tag_list.tagdic

# write the path of the world file
working_dir = "/media/j/T7/git/hanwha/src/natural_enviroments/worlds/"
input_file = "lakepark_test3"

output_file = input_file + "_tagged"
input_file = working_dir + input_file + ".world"
output_file = working_dir + output_file + ".world"

if os.path.isfile(output_file):
    os.remove(output_file)

with open(input_file, 'r') as f:
    world = f.read()
    for key, value in tagdic.items():
        world = world.replace(key, value)
    f.close()
with open(output_file, 'w') as f:
    f.write(world)
    f.close()
