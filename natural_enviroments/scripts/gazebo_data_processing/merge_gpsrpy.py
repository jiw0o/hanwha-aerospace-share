import os

# Set your directories
root_direc = 'gazebo_kitti/raw_data'
day_direc ='2024_08_12'
bag_list = ['lakepark1', 'lakepark2', 'lakepark3', 'lakepark4']

def merge_gpsrpy(file1_path, file2_path, output_path):
    # Ensure output path exists
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    # List of index files
    file1_indices = sorted([f for f in os.listdir(file1_path) if f.endswith('.txt')])
    file2_indices = sorted([f for f in os.listdir(file2_path) if f.endswith('.txt')])

    # Ensure both directories have the same number of files
    if len(file1_indices) != len(file2_indices):
        print("The number of files in both directories do not match.")
        return

    # Merge files
    for index in file1_indices:
        with open(os.path.join(file1_path, index), 'r') as file1, open(os.path.join(file2_path, index), 'r') as file2:
            file1_content = file1.readline().strip()
            file2_content = file2.readline().strip()
            merged_content = file1_content + " " + file2_content
            
            output_file = os.path.join(output_path, index)
            with open(output_file, 'w') as output:
                output.write(merged_content)
        print("{} {} GPS and LiDAR merged".format(bag, index))

if __name__ == "__main__":
    for bag in bag_list:
        gps_path = os.path.join(root_direc, day_direc, bag, 'oxts', 'data_gps')
        rpy_path = os.path.join(root_direc, day_direc, bag, 'oxts', 'data_rpy')
        output_path = os.path.join(root_direc, day_direc, bag, 'oxts', 'data')
        merge_gpsrpy(gps_path, rpy_path, output_path)
