# PID implementation Turtlesim

### Task
Model the motion from initial pose (<!-- $\bar{x}, \bar{y}, \bar{\theta}$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbar%7Bx%7D%2C%20%5Cbar%7By%7D%2C%20%5Cbar%7B%5Ctheta%7D">) to goal pose (<!-- $\bar{x\prime}, \bar{y\prime}, \bar{\theta\prime}$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cbar%7Bx%5Cprime%7D%2C%20%5Cbar%7By%5Cprime%7D%2C%20%5Cbar%7B%5Ctheta%5Cprime%7D">) in the following 3 parts as shown in figure and use PID controllers separately on the 3 parts:
1. Turn by <!-- $\delta_{rot1}$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cdelta_%7Brot1%7D">
2. Move straight for <!-- $\delta_{trans}$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cdelta_%7Btrans%7D">
3. Turn by <!-- $\delta_{rot2}$ --> <img style="transform: translateY(0.1em); background: white;" src="https://render.githubusercontent.com/render/math?math=%5Cdelta_%7Brot2%7D">

https://user-images.githubusercontent.com/63636498/138709362-928f9875-b986-4900-a582-c11f4ee95a43.mp4

## Usage

Clone this repository in catkin source directory
```
cd ~/catkin_ws/src
git clone https://github.com/ABD-01/ros_pid.git
```

Build the package in catkin workspace
```
cd ~/catkin_ws && catkin_make
```

To start ROS Master
```
roscore
```

To start a turtlesim simulator
```
rosrun turtlesim turtlesim_node
```

Run the PID simulator with command line arguments for final pose.
```
rosrun ros_pid turtlesim_pid.py [-h] -x X -y Y -t THETA

Input final pose

optional arguments:
  -h, --help            show this help message and exit
  -x X, --x X           Final x position
  -y Y, --y Y           Final y position
  -t THETA, --theta THETA
                        Final angle (in degrees)
```
For eg:
```
rosrun ros_pid turtlesim_diff.py -x 1 -y 1 -t 120
```

Start the rqt graph for visualization
```
rqt_plot /plot/error/data[0] /plot/error/data[1]
```
![results](/assets/rospid.gif)

## Tuning coefficients Kp, Ki, Kd

Custom coefficient values and loop rate can be passed while creating the PID class instance.

```py
  pid = PID(
      final_pose,
      k=K(6, 0.0, 1),
      loop_rate=10,
      v0 = 1
  )
```