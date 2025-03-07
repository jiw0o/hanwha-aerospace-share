import os
import tag_list

model_class_dic = tag_list.model_class_dic
class_tag_dic = tag_list.class_tag_dic
tag_dic = tag_list.tag_dic

# write the path of the world file
working_dir = "/media/j/T7/git/hanwha/src/natural_enviroments/worlds/"
input_file = "lakepark_test3"

output_file = input_file + "_tagged"
input_file = working_dir + input_file + ".world"
output_file = working_dir + output_file + ".world"

if os.path.isfile(output_file):
    os.remove(output_file)

def process_gazebo_world(input_file, output_file):
    visual = False
    current_class = None

    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        # replace     world = f.read()
        # for key, value in tagdic.items():
        # world = world.replace(key, value)
        world = infile.read()
        for key, value in tag_dic.items():
            world = world.replace(key, value)
        
        lines = world.splitlines()
        for line in lines:
            stripped_line = line.strip()
            
            if visual == False:
                # Detect visual tag
                if "<visual " in stripped_line and "name='visual'>" in stripped_line:
                    visual = True
                outfile.write(f"{line}\n")  # Write the line as it is
                continue

            elif visual:
                if current_class is None:
                    # Detect uri tag for class
                    if "<uri>" in stripped_line:
                        model = stripped_line.split("model://")[1].split("</")[0]
                        current_class = model_class_dic[model]
                        if current_class not in ["rock", "foilage", "trunk"]:
                            visual = False
                            current_class = None
                    outfile.write(f"{line}\n")  # Write the line as it is
                    continue
                elif current_class:
                    # Detect cast_shadows tag
                    if "<cast_shadows>1</cast_shadows>" in stripped_line:
                        # Insert material based on the current class
                        outfile.write(f"{line}")
                        outfile.write(f"{class_tag_dic[current_class][0]}\n")
                        continue
                    # Detect laser_retro tag
                    elif "<laser_retro>" in stripped_line and "</laser_retro>" in stripped_line:
                        key = stripped_line.split(">")[1].split("<")[0]
                        line = line.replace(key, class_tag_dic[current_class][1])
                        outfile.write(f"{line}\n")
                        visual = False
                        current_class = None
                        continue
                    outfile.write(f"{line}\n")


process_gazebo_world(input_file, output_file)
