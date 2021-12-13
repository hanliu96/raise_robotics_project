# raise_robotics_project
Take home project for the interview of Raise Robotics: use the Cartesian planning function to move the robot arm in a circle.

Open two shells. Start RViz and wait for everything to finish loading in the first shell:

```` 
roslaunch panda_moveit_config demo.launch
````

Now run the Python code directly in the other shell using rosrun:
```` 
rosrun moveit_tutorials move_group_python_interface_tutorial.py
````

