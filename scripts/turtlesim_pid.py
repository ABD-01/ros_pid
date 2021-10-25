#!/usr/bin/env python3

import math
import time
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import argparse
from namedlist import namedlist
from copy import deepcopy

E = namedlist("Error", ["rot1", "trans", "rot2"])
K = namedlist("K", ["kp", "ki", "kd"])

class Error(E):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def __add__(self, other):
        return Error(self.rot1+other.rot1, self.trans+other.trans, self.rot2+other.rot2)
    
    def __sub__(self, other):
        return Error(self.rot1-other.rot1, self.trans-other.trans, self.rot2-other.rot2)
    
    def __str__(self):
        return f"Error(rot1: {self.rot1:.2f}, trans: {self.trans:.2f}, rot2: {self.rot2:.2f})"
    

class PID:
    def __init__(
        self,
        final_pose: Pose,
        Krot1=K(4, 0, 2),
        Ktrans=K(0.5, 0.001, 0.02),
        Krot2=K(1, 0.0, 0.01),
        loop_rate=10,
    ):

        self.current_pose = None
        self.error = Error(0, 0, 0)
        self.final_pose = final_pose

        rospy.init_node("turtlesim_pid", anonymous=True)
        self.loop_rate = rospy.Rate(loop_rate)
        self.subscriber = rospy.Subscriber(
            "/turtle1/pose", Pose, callback=self.callback
        )
        # time.sleep(4)
        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.tracker = rospy.Publisher("/plot/error", Float32MultiArray, queue_size=10)
        self.Krot1, self.Ktrans, self.Krot2 = Krot1, Ktrans, Krot2

    def callback(self, pose_msg):
        self.current_pose = pose_msg
        print("Pose cb(x: {:.2f}  y: {:.2f}  theta: {:.2f})".format(
                self.current_pose.x, self.current_pose.y, self.current_pose.theta))

    def move(self):

        vel_msg = Twist()
        error_old = Error(0,0,0)
        E = Error(0,0,0)
        # tstart = rospy.Time.now().to_sec()
        while True:

            # Clculating Error
            slope = math.atan2(
                self.final_pose.y - self.current_pose.y,
                self.final_pose.x - self.current_pose.x
            )
            self.error.rot1 = slope - self.current_pose.theta
            self.error.trans = math.dist(
                [self.final_pose.x, self.final_pose.y],
                [self.current_pose.x, self.current_pose.y],
            )
            self.error.rot2 = self.final_pose.theta - self.current_pose.theta # - slope
            print(self.error)
            error0 = self.error - error_old
            E = E + self.error

            ep = Float32MultiArray()
            ep.data = [self.error.trans, self.error.rot1]

            # Calculating Velocity
            angular_velocity1 = self.error.rot1 * self.Krot1.kp + E.rot1 * self.Krot1.ki + error0.rot1 * self.Krot1.kd
            linear_velocity = self.error.trans * self.Ktrans.kp + E.trans * self.Ktrans.ki + error0.trans * self.Ktrans.kd
            angular_velocity2 = self.error.rot2 * self.Krot2.kp + E.rot2 * self.Krot2.ki + error0.rot2 * self.Krot2.kd

            error_old = deepcopy(self.error)
            self.loop_rate.sleep()

            # tnow = rospy.Time.now().to_sec()
            # rot1 = (tnow-tstart)*self.current_pose.angular_velocity
            # trans = (tnow-tstart)*self.current_pose.linear_velocity
            # rot2 = (tnow-tstart)*self.current_pose.angular_velocity

            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity1

            if self.error.trans < 0.05:
                vel_msg.linear.x = 0
            # if abs(self.error.rot1) < 0.01:
                vel_msg.angular.z = angular_velocity2
                ep.data[-1] = self.error.rot2
                print("Almost there")
                if (abs(self.error.rot2) < 0.01):
                    vel_msg.angular.z = 0
            else:
                error0.rot2 = 0
                E.rot2 = 0

            self.tracker.publish(ep)
            # print(f"Linear Velocity: {vel_msg.linear.x}, Angular velocity: {vel_msg.angular.z}")
            self.publisher.publish(vel_msg)

            if vel_msg.linear.x == 0 and vel_msg.angular.z == 0:
                print("Destination reached")
                exit()
                break


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
        time.sleep(2)
        pid.move()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
