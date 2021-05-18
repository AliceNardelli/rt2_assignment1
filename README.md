
# First Assignment of Research Trach 2 course - ros2 branch

# Packages needed

In this part of the assignment in order to create a bridge between ROS and ROS2 and make nodes communicates (ros1_bridge)[https://github.com/ros2/ros1_bridge] package is required.


## Behaviour of the software architecture

The main task is to make communicate ROS nodes **user_interface.py**, **go_to_point.py** with ROS2 nodes **state_machine.cpp**, **position_service.cpp**  through a ROS bridge maintaing the behaviour of the system unchanged. 
The **rt2_assignment1** with ros2 branch is a ROS2 CMake package. The c++ nodes are implemented as components and uploaded inside a container. To do that they have been implemented as Classes. 


### srv folder

It contains three custome services, the three services are defined identical to services inside **rt2_assignment1** with main branch services: 
1. **RandomPosition** 
2. **Command** 
3. **Position** 



### launch folder

It contains:
1. **all_launch.py**: a launch file to run the container and the two executables.
 

### src folder

Inside this folder there are the c++ code files.

1. **position_service.cpp**: this is a service server node that given as a request a minimum and a maximum value of x and y coordinate both return as response a random value for x, y and theta.
.
2. **state_machine.cpp**: the state machine is a service server that can be activated or deactivated managing a boolean variable *start* set as request by the **user_interface.cpp** node. The node is a service client of the **/position_server** and of **/go_to_point**. It asks for a random position and if it is activated it set the response values as **/go_to_point** request. 


### mapping_rule.yaml

It is a file containing the mapping rule used to compile the bridge. 


## How to run the scene.

1. Go inside a folder where the file to source ROS, ROS2 and the bridge are contained
2. execute
 
   ```
     ./all_simulation.sh

   ```

alternatively

1. inside a ROS terminal run :

   ```
     roslaunch rt2_assignment1 sim2.launch
     
   ```
2. inside a ROS12 terminal run :

  ```
     ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
     
  ```
3. inside a ROS2 terminal run :

  ```
     ros2 launch rt2_assignment1 all_launch.py
     
  ```
   

   




