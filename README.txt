Authors: Jorge Espinar, Marzio Lunghi

Most advanced task is: Task 3 
To run task 3 do:

1. Build the project using:
colcon build --packages-select assignment2

2. In a new terminal start the bridge:
ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0

3. In a new terminal (after source ~/dev_ws/install/setup.bash), run:
ros2 launch assignment2 controller.launch.xml thymio_name:=thymio0

Note: you have to load the wall scene in Coppelia

For the bonus
Repeat setp 1. and 2. above and then in a new terminal (after source ~/dev_ws/install/setup.bash) run:
ros2 launch assignment2 controller.launch.xml thymio_name:=thymio0 exec_name:=bonus

By default 'exec_name' has value 'task3'

Note: you have to open the 'HW2 scene' scene in Coppelia 

Note:
All the tasks have been implemented on top of thymio_example [https://github.com/EliaCereda/thymio_example], more precisely the controller class in controller_node.py was used.
Plus, the euclidian distance function was copied from previous assginment.

Videos link = *insert_link*
