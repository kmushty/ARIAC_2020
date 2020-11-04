# Ariac

# Run the folowing commands:
Open a terminal
```
cd ariac_ws/src
git clone https://github.com/mesneym/Ariac.git
git checkout tweaking
cd ..
catkin build
source devel/setup.bash
roslaunch rwa4_3 rwa4.launch load_moveit:=True
```

In another terminal:
```
cd ariac_ws
source devel/setup.bash
rosrun rwa4_3 rwa3_node
```