# Hanwha Aerospace Project

## 1. Build & Setting

### [Install packages]

    $ sudo apt install graphviz graphviz-dev
    $ pip install pygraphviz


### Clone this repository in the workspace/src.

### Reinstall ompl
**[Note!] reinstall ompl within your workspace using ThirdParty.zip in this repo.**

    $ export WS_DIR=(The workspace path where you cloned this repository.)
    
    $ export LD_LIBRARY_PATH=${WS_DIR}/src/hanwha-aerospace-share/planning_module/ThirdParty/ompl/share:$LD_LIBRARY_PATH
    
    "Extract ThirdParty.zip in WS_DIR"

    $ cd ${WS_DIR}/ThirdParty/ompl/build

    $ cmake ..

    $ sudo make -j 16 install

    $ sudo cp /usr/local/share/libompl.so* ${WS_DIR}/src/hanwha-aerospace-share/planning_module/ThirdParty/ompl/share/ 

    $ sudo cp -r /usr/local/include/ompl-1.6/ompl ${WS_DIR}/src/hanwha-aerospace-share/planning_module/ThirdParty/ompl/include

### Download the map file from the server
    $ mkdir ~/{$TRIP_PACKAGE}/map
    $ cd ~/{$TRIP_PACKAGE}/map
    $ chmod +x download_maps.sh
    $ ./download_maps.sh

-----------------------

## 2. Execution

### Load Environment & Robot 
    (TERMINAL 1)
    $ roslaunch natural_environments create_lakepark.launch
    
    (TERMINAL 2)
    $ roslaunch natural_environments add_husky_lakepark.launch

### Subscribe Odometry & Publish Local Maps
    (TERMINAL 3)
    $ roslaunch trip_loader load_gazebo.launch


### Planning
    (TERMINAL 4)
    $ export WS_DIR=(The workspace path where you cloned this repository.)

    $ export LD_LIBRARY_PATH=${WS_DIR}/src/hanwha-aerospace-share/planning_module/ThirdParty/ompl/share:$LD_LIBRARY_PATH

    $ catkin_make && roslaunch planning_module main.launch

### MPPI Ctrl
    (TERMINAL 5)
    $ roslaunch mppi_controller mppi_ctrl_husky.launch


### Test Package

> #### A test node that iterates through all test cases. 
    (TERMINAL 6)
    $ rosrun test_pkg hanwha_loop_test.py
    Type 'p' in terminal to automatically run the experiment.
    
> #### A test node that performs a specific test case you want.
    (TERMINAL 6)
    $ rosrun test_pkg hanwha_user_test.py
    Type 'p' into the terminal to start the node.
    Type the index of the desired test case(1, 2, 3, ...) into the terminal.
    You can try a another test case every time the previous one is completed.

-----------------------




