We present three different maze exploring techniques:
1. Simple follow the right wall until you reach the end of the maze (can get stuck)
2. Path planning algorithm (assumens all positions are known) which compute the optimal path before moving the robot
3. Path planning algorithm while exploring (assumens only the dimension of the maze, starting point coordinates and end point coordinates)

First step:
    load the scene in coppelia (can be found inside './scene/)

Second step (start the bridge): 
    ros2 launch thymioid main.launch device:="tcp:host=localhost;port=33333" simulation:=True name:=thymio0

Second step (start the thymio and solve maze):
    To run 1.:
        ros2 launch assignment2 controller.launch.xml thymio_name:=thymio0 exec_name:=right

    To run 2.:

        ros2 launch assignment2 controller.launch.xml thymio_name:=thymio0 exec_name:=solver algo_name:=<name of the algorithm>

        Notice that <name of the algorithm> must be replaced by:
            a. DFS
            b. BFS
            c. DJ (Dijkstra)
            d. BC (BiconnectedBFS)
    
    To run 3.:
        ros2 launch assignment2 controller.launch.xml thymio_name:=thymio0 

