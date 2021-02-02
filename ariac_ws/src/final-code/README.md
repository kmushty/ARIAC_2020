# Ariac

# Run the folowing commands:
Open a terminal
```
cd ariac_ws/src
git clone https://github.com/mesneym/Ariac.git
cd ..
catkin build
source devel/setup.bash
roslaunch final-code rwa5.launch load_moveit:=True
```

In another terminal:
```
cd ariac_ws
source devel/setup.bash
rosrun final-code rwa3_node
```
In order to view the doxygen documentation open webpage using index.html present in doxygen/html folder in final-code package.
