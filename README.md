# Robotics-Coursework
coursework for robotics maze solver

(words 99)
the program is subscribed to 3 publishers the laser scan, the odometry and the image_raw. It uses the laser scan to sense if there is a wall in front of it, to sense if there is a junction either side and to center itself on the wall. It uses the odometry for turning set angles once its reached a junction and the image is to detect the red, blue and green squares. It runs a separate function called runner which controls the turning so that the program can have the laser scan and the odometry updated during the turns

to run the program ensure that the gazebo and roscore are running and that the code will have access to them and run the program from vs code. The robot should be able to go from the initial starting position.
