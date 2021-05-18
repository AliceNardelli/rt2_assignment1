
#First Assignment of Research Trach 2 course - main branch


##Behaviour of the software architecture

When the simulation start the user can insert 1 to make the robot start. The robot continuosly reaches random generated poses since the user presses 0. When this happens the robot firstly reach the last generated random pose then it stops.


###Srv folder

It contains three custome services:
1. **RandomPosition** returns as reply two random x and y coordinates given as request the maximum and minimum value for each one.
2. **Command** it takes a string as request and return a boolean as reply.
3. **Position** it takes as input a position to reach and as output a boolean.


###Urdf folder

It contains the description of the world simulated through gazebo and the model of the robot.

###launch folder

It contains:
1. **sim.launch**: a launch file to start the overall simulation of world and robot and all the nodes. 
2. **sim2.launch**: a launch file containing the simulation only the python nodes used in the second point of the assignment. I
3. **simVrep.launch**: a launch file to start the four nodes, it is used to in the third point of the assignment to interact with Vrep. 

###src folder

Inside this folder there are the c++ code files.

1. **position_service.cpp**: this is a service server node that given as a request a minimum and a maximum value of x and y coordinate both return as response a random value for x, y and theta.
.
2. **state_machine.cpp**: the state machine is a service server that can be activated or deactivated managing a boolean variable *start* set as request by the **user_interface.cpp** node. The node is a service client of the **/position_server** and of **/go_to_point**. It asks for a random position and if it is activated it set the response values as **/go_to_point** request. 

###scripts folder

Inside this folder there are python code files.

1. **go_to_point.py**: this is a service server node that implement the go to point point behaviour. 

2. **user_interface.py**: this node implements the user interface, ask to user to insert 1 for starting the simulation and 0 to stop the robot. 

##Packages needed

In order to generate Coppeliasim scene and make it integrated with ROS [Coppeliasim](https://www.coppeliarobotics.com), [SimExtROS](https://github.com/CoppeliaRobotics/simExtROS) and (ros1_bridge)[https://github.com/ros2/ros1_bridge] packages has been used.


##VRep scene

The file **sceneVrep.ttt** contains the scene to upload on CoppeliaSim. A pioneer_ctrl robot is controlled through the four nodes of the assignment. This is possible thanks to the fact that the simulation is a fifth node which interacts with ROS nodes. In the childscripts a publisher to /odom topic has been declarated. This is used for publishing the actual position of the robot subscribed in node **go_to_point.py** . Moreover a subscriber to /cmd_vel is decleclared to retrieve the velocity of the robot set in the same node and actuate the motors.

##How to run the scene.

1. Open Coppeliasim in a sourced ROS terminal.
2. Load the scene on CoppeliaSim.
3. Launch on another terminal:
   '''
       roslaunch rt2_assignment1 simVrep.launch
   '''








