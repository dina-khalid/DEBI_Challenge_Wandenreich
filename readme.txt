Project Name: Turtlebot3 Waffle_pi Simulation
Package Name: nav_node
Source File: navigation_node.py

Launch Command:
roslaunch turtlebot3_debipkg turtlebot3_depiworld.launch

Steps to Run:
1. Navigate to the project directory in the terminal.
2. Run catkin_make to build the packages and dependencies.
3. In each new terminal, run the following commands to set up the environment:
    source devel/setup.bash
    export TURTLEBOT3_MODEL=waffle_pi
4. In the first terminal, run the following command to start the ROS core:
    roscore
5. In the second terminal, navigate to the project directory and run the following command to launch the world in Gazebo:
    roslaunch turtlebot3_debipkg turtlebot3_depiworld.launch
6. In the third terminal, navigate to the package directory and run the following command to start the navigation node:
    rosrun nav_node navigation_node.py
7. The Turtlebot3 Waffle_pi robot will start moving the balls from one side of the playground to another.
8. Once the robot reaches its destination, the navigation node will print the message "Navigation completed successfully" in the terminal.

Notes:
- Make sure that all the required dependencies are installed before running the simulation.
- This simulation assumes that the navigation stack is properly configured and calibrated for the Turtlebot3 Waffle_pi model.