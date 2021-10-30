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

K = namedlist("K", ["kp", "ki", "kd"])

def step_funt(x, tolerance=0.1):
    return 1 * (x > tolerance)

class PID:
    def __init__(
        self,
        final_pose: Pose,
        k=K(6, 0.0, 1),
        loop_rate=10,
        v0 = 1
    ):

        self.v0 = v0
        self.current_pose = None
        self.error = 0 #Error(0, 0)
        self.final_pose = final_pose

        rospy.init_node("turtlesim_pid", anonymous=True)
        self.loop_rate = rospy.Rate(loop_rate)
        self.subscriber = rospy.Subscriber(
            "/turtle1/pose", Pose, callback=self.callback
        )
        # time.sleep(4)
        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.tracker = rospy.Publisher("/plot/error", Float32MultiArray, queue_size=10)
        self.k = k

    def callback(self, pose_msg):
        self.current_pose = pose_msg
        print("Pose cb(x: {:.2f}  y: {:.2f}  theta: {:.2f})".format(
                self.current_pose.x, self.current_pose.y, self.current_pose.theta))

    def move(self):

        vel_msg = Twist()
        error_old = 0
        E = 0
        # tstart = rospy.Time.now().to_sec()
        while True:

            y_error = self.final_pose.y - self.current_pose.y
            x_error = self.final_pose.x - self.current_pose.x
            dist_error = math.sqrt(y_error**2 + x_error**2)

            # Clculating Error
            slope = math.atan2(y_error, x_error)

            # self.error = slope - self.current_pose.theta
            # self.error = self.final_pose.theta - self.current_pose.theta
            self.error = step_funt(dist_error) * slope + (1-step_funt(dist_error)) * self.final_pose.theta - self.current_pose.theta
            
            self.error = math.atan2(math.sin(self.error), math.cos(self.error))

            error0 = self.error - error_old
            E = E + self.error

            ep = Float32MultiArray()
            ep.data = [dist_error, self.error]

            # Calculating Velocity
            angular_velocity = self.error * self.k.kp + E * self.k.ki + error0 * self.k.kd

            error_old = deepcopy(self.error)
            self.loop_rate.sleep()

            # tnow = rospy.Time.now().to_sec()
            # rot = (tnow-tstart)*self.current_pose.angular_velocity
            # trans = (tnow-tstart)*self.current_pose.linear_velocity

            vel_msg.linear.x = self.v0
            vel_msg.angular.z = angular_velocity

            if (dist_error < 0.1):
                vel_msg.linear.x = 0
                
                print("Almost there")
                print(self.error)
                if (abs(self.error) < 0.05):
                    vel_msg.angular.z = 0

            self.tracker.publish(ep)
            # print(f"Linear Velocity: {vel_msg.linear.x}, Angular velocity: {vel_msg.angular.z}")
            self.publisher.publish(vel_msg)

            if vel_msg.linear.x == 0 and vel_msg.angular.z == 0:
                print("Destination reached")
                # exit()
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
