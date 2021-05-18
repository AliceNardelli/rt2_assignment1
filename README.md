   
# First Assignment of Research Trach 2 course - action branch

## List of required packages

[Actionlib](http://wiki.ros.org/actionlib) is a package that allows to implements an action server. Action differently to service aren't instantaneous but that have a duration in time. Action client has the possibility to get some feedback when action is happening and cancel the action before the conclusion if needed. The action is defined as a file *.action* inside a folder *Action* contained in the package.


## System description
The overall behaviour of the system is the same of the main branch. The main difference of this branch is that **go_to_point** node has been implemented as an action server whereas the **state_machine**  and **user_interface** are action clients.

 
### Action folder
It contains the definition of the implented action.
The goal is the position that robot has to reach, the result is a boolean variable whereas the the feedback is the actual position of the robot.

### Srv folder

The two service messages are defined as in branch main

1. **RandomPosition** 
2. **Command** 

### Urdf folder
It contains the description of the world and of the robot.

### launch folder
It contains the launch file to launch **sim.launch** the overall simulation.

### src folder

Inside this folder there are the c++ code files.

1.**position_service.cpp**: the implementation of this node is the same of the of the one in the main branch
.
2.**state_machine.cpp**: the structure of the code of this node has been maintained the same of the one in the branch main. The only difference is that instead of a service client of */go_to_point* here has been implemented a action client of */go_to_point*. It send a goal position to the action server and wait since robot reaches this position.

## scripts folder

1. **go_to_point.py**: this is a action server node that implement the reach a point behaviour. The node has been structured a class and the server is an object of the class GoalReachingAction. Inside the constructor of the class the action server is defined. I would like to emphasize the role of the callback of the action server. As the fact that it implements the machine state of this node and control the overall behaviour when robot is moving. In particular through the structure of the action it is possible to continuosly publish as feedback the actual position of the robot.

2. **user_interface.py**: this node implements the user interface, the only difference with the main is that there is defined an action client of GoalReachingAction in such a way to cancel the goal if the user press 0 even if the goal has not been already reached. When the action is canceled the robot is stop instantanously through a Twist message.

## How to run the code

To run the code is necessary to run this line of code:

```
roslaunch rt2_assignment1 sim.launch

```
