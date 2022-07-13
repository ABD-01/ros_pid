#!/usr/bin/env python3

import math
import time

from numpy.linalg.linalg import norm
import rospy
# import numpy as np
from numpy import array, sin, cos, dot, arctan2, linalg, ones_like, linspace, arange, vstack
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import argparse
from namedlist import namedlist
from copy import deepcopy, error
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


K = namedlist("K", ["kp", "ki", "kd"])

class Hermite:

    H = array([[2,-2,1,1],[-3,3,-2,-1],[0,0,1,0],[1,0,0,0]])

    def __init__(self, P0, P1, P_dot0, P_dot1) -> None:
        P = array([P0, P1, P_dot0, P_dot1])
        assert P.shape[0] == 4
        self.A = dot(self.H, P)
        
    def __call__(self, time):
        Time = vstack([time**3, time**2, time, ones_like(time)])
        return dot(Time.T, self.A)

    def view_traj(self):
        # t = arange(0, 1, 0.05)
        # traj = self.__call__(t)
        # plt.plot(traj[:,0], traj[:,1])
        # plt.show()
        # print(traj)

        fig, ax = plt.subplots()
        xdata, ydata = [], []
        ln, = plt.plot([], [], 'ro-')

        def init():
            ax.set_xlim(0, 11)
            ax.set_ylim(0, 11)
            return ln,

        def update(frame):
            traj = self(frame).squeeze()
            xdata.append(traj[0])
            ydata.append(traj[1])
            ln.set_data(xdata, ydata)
            return ln,

        ani = FuncAnimation(fig, update, frames=linspace(0, 1, 20),
                            init_func=init, blit=True)
        plt.show()

        

class PID:
    def __init__(
        self,
        final_pose: Pose,
        k=K(0.9, 0.01, 0.5),
        ka=K(1.6, 0.0, 0),
        loop_rate=10,

    ):

        self.current_pose = None
        self.error = 0
        self.final_pose = final_pose

        rospy.init_node("turtlesim_pid", anonymous=True)
        self.loop_rate = rospy.Rate(loop_rate)
        self.subscriber = rospy.Subscriber(
            "/turtle1/pose", Pose, callback=self.callback
        )
        time.sleep(4)
        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.tracker = rospy.Publisher("/plot/error", Float32MultiArray, queue_size=10)
        self.k = k
        self.ka = ka

        s = 20
        self.hermite = Hermite(
            [self.current_pose.x, self.current_pose.y], # start (x,y)
            [self.final_pose.x, self.final_pose.y],     # final (x,y)
            [s * cos(self.current_pose.theta), s * sin(self.current_pose.theta)], # start (x', y')
            [s * cos(self.final_pose.theta), s * sin(self.final_pose.theta)]      # final (x', y')
        )
        self.hermite.view_traj()

    def callback(self, pose_msg):
        self.current_pose = pose_msg
        # print("Pose cb(x: {:.2f}  y: {:.2f}  theta: {:.2f})".format(
        #         self.current_pose.x, self.current_pose.y, self.current_pose.theta))

    def get_error(self, p_desired):
        diff = p_desired - array([self.current_pose.x, self.current_pose.y])
        distance = linalg.norm(diff)
        angle = arctan2(diff[1], diff[0]) - self.current_pose.theta
        return float(distance), float(angle)

    def move(self):
        vel_msg = Twist()
        error_old = 0
        E = 0

        # t0 = rospy.Time.now().to_sec()
        time = 0
        p_desired = self.hermite(time)
        tol = 1e-1
        while True:
            # t1 =  rospy.Time.now().to_sec()
            # time = t1 - t0

            print(f"Planed path {time:.2f}: [{self.current_pose.x, self.current_pose.y}] -> {p_desired}")
            error, error_a = self.get_error(p_desired.squeeze())
            error_a = math.atan2(math.sin(error_a), math.cos(error_a))

            # if error <= tol:
            if time <= 1:   
                time += 0.01
            E = 0
            error_old = 0
            p_desired = self.hermite(time)

            # print("Error:", error, error_a)
            linear_speed = error * self.k.kp + E * self.k.ki + error_old * self.k.kd
            angular_speed  = error_a * self.ka.kp

            error_old = error
            E += error

            print(linear_speed, angular_speed)
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed

            # if time > 1.0:
            final_errors = self.get_error(self.hermite(0.95).squeeze())
            if final_errors[0] < 1e-1 and abs(final_errors[1]) < 1e-1:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                print("STOPPED")
            else:
                print(f"Final Errors {final_errors}")

            self.publisher.publish(vel_msg)
            self.loop_rate.sleep()



if __name__ == "__main__":
    try:
        parser = argparse.ArgumentParser(description="Input final pose")
        parser.add_argument(
            "-x", "--x", type=float, required=True, help="Final x position"
        )
        parser.add_argument(
            "-y", "--y", type=float, required=True, help="Final y position"
        )
        parser.add_argument(
            "-t", "--theta", type=float, required=True, help="Final angle (in degrees)"
        )

        args = parser.parse_args()

        final_pose = Pose()
        final_pose.x = args.x
        final_pose.y = args.y
        final_pose.theta = math.radians(args.theta)

        pid = PID(final_pose)
        # time.sleep(2)
        pid.move()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
