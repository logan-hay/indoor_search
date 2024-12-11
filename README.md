### TO INSTALL PACKAGE FOR ASSIGNMENT 

1. Set up environment variables for ROS. Make sure to replace '/home/rpi/shared' with your own shared folder location
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Fork your own repository of f23_robotics (using web interface)

3. Clone your fork
<pre>
git clone <your github url for this repository>
</pre>

4. Make the package (for python, it really just installs the files)
<pre>
cd f24_robotics
colcon build
</pre>

5. Set up variables to use the package you just created
<pre>
source install/setup.bash
</pre>

6. Start webots simulation with connect back to ROS in the virtual machine
<pre>
ros2 launch indoor_search launch.py
</pre>

### RUN CARTOGRAPHER
In a new terminal:
<pre>
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
</pre>

### SAVE MAP
While rviz is still running, In a new terminal:
<pre>
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map
</pre>

### RUN CONTROLLER
In a new terminal:
<pre>
cd indoor_search/
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run indoor_search indoor_search
</pre>

### GENERATE RESULTS
Set the path at the top of gen_results.py to match your map file then, while inside the results directory, run:
<pre>
python3 gen_results.y
</pre>

