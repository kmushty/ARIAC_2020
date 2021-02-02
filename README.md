# Agile Robotics for Industrial Automation Competition(ARIAC):
## Introduction:
ARIAC requires completion of series of tests centered in an industrial scenario that are based around order fulfillment. More information regarding ARIAC can be found [here](https://github.com/usnistgov/ARIAC/blob/master/wiki/documentation/documentation.md). This repository highlights the completion of agility challenges.


## Run the folowing commands:
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

## Output:
### Faulty Part Agility Challenge:
![](gif/faulty_part.gif)


### Faulty Gripper Agility Challenge:
![](gif/faulty-gripper.gif)

### Flip Part Agility Challenge:
![](gif/flip-part.gif)

## Important Milestone:
### Dynamic Preset Location:
![](gif/dynamic_preset.gif)

For more information regarding the implementation and output please read the report [here](https://github.com/mesneym/Ariac/blob/master/report/Report.pdf).