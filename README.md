# e-yantra
Me and Deepak Sai Majjiga created the work

task_1:
This is a ROS package, in which we have created a URDF file for a custom holonomic drive bot. Written a controller file in python to move the bot in a gazebo world. We made a controller file for bot which moves according to the goals given in the files 'goals.json'.

task_2:
In this we created a ROS package,in which used overhead camera to localize the bot. An aruco marker has been set up on the bot which is detected by the overhead camera using OpenCV. The file feedback.py publishes the coordinates of the bot to the corresponding rostopic. And controller.py sends the velocity commands for each wheel of the holonomic bot according the goals and current postion of the bot.
