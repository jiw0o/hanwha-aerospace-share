# how to open a bin file
# how to compare two bin files
import os

file1_dir = 'gazebo_kitti/raw_data/2024_07_02/lakepark1/velodyne_points/data'
file2_dir = 'gazebo_kitti'

# i:0~10
for i in range(10):
    file1 = os.path.join(file1_dir, "{:010d}.bin".format(i))
    file2 = os.path.join(file2_dir, "{:010d}.bin".format(i))

    with open(file1, 'rb') as f1, open(file2, 'rb') as f2:
        data1 = f1.read()
        data2 = f2.read()

        if data1 == data2:
            print(f"{file1} and {file2} are the same")
        else:
            print(f"{file1} and {file2} are different")