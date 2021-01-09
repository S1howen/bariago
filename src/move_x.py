#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import math
import sys
import rospy
import time
from geometry_msgs.msg import Twist


class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class Move_from_x_to_y():
    def __init__(self, asked_movement):
        self._pub_cmd = rospy.Publisher('key_vel', Twist)
        self._hz = rospy.get_param('~hz', 100)
        self.rotation_rate = 0
        self.moveTime = 0
        self.start_serv = 2
        #self.nitu_gradur = 5.7
        if asked_movement == "right":
            self.right()
        elif asked_movement == "left":
            self.left()
        elif asked_movement == "straight":
            self.straight(1)
        elif asked_movement == "start_rum":
            self.left()
            time.sleep(1)
            self.straight(1)
            time.sleep(1)
            self.right()
        elif asked_movement == "rum_cola":
            self.left()
            time.sleep(1)
            self.straight(2)
            time.sleep(1)
            self.right()
        elif asked_movement == "cola_serv":
            self.right()
            time.sleep(1)
            self.straight(3)
            time.sleep(1)
            self.right()
            self.straight(self.start_serv)
        elif asked_movement == "serv_start":
            self.right()
            time.sleep(1)
            self.right()
            time.sleep(1)
            self.straight(self.start_serv)
        elif asked_movement == "start_wis":
            self.left()
            time.sleep(1)
            self.straight(2)
            time.sleep(1)
            self.right()
        elif asked_movement == "wis_cola":
            self.left()
            time.sleep(1)
            self.straight(1)
            time.sleep(1)
            self.right()
        elif asked_movement == "start_gin":
            self.left()
            time.sleep(1)
            self.straight(4)
            time.sleep(1)
            self.right()
        elif asked_movement == "gin_tonic":
            self.left()
            time.sleep(1)
            self.straight(1)
            time.sleep(1)
            self.right()                   
        elif asked_movement == "tonic_serv":
            self.right()
            time.sleep(1)
            self.straight(5)
            time.sleep(1)
            self.right()
            self.straight(self.start_serv)
        elif asked_movement == "start_beer":
            self.left()
            time.sleep(1)
            self.straight(6)
            time.sleep(1)
            self.right()
        elif asked_movement == "beer_serv":
            self.right()
            time.sleep(1)
            self.straight(6)
            time.sleep(1)
            self.right()
            self.straight(self.start_serv)
        elif asked_movement == "start_wine":
            self.left()
            time.sleep(1)
            self.straight(7)
            time.sleep(1)
            self.right()        
        elif asked_movement == "wine_serv":
            self.right()
            time.sleep(1)
            self.straight(7)
            time.sleep(1)
            self.right()
            time.sleep(1)
            self.straight(self.start_serv)
        elif asked_movement == "wis_lime":
            self.left()
            time.sleep(1)
            self.straight(7)
            time.sleep(1)
            self.right()
        elif asked_movement == "lime_serv":
            self.right()
            time.sleep(1)
            self.straight(9)
            time.sleep(1)
            self.right()
            self.straight(self.start_serv)  
        #self._backward_rate = rospy.get_param('~backward_rate', 0.3)
    def right(self):
        self.moveTime = 5.4
        self._forward_rate = rospy.get_param('~forward_rate', 0.0)
        self._rotation_rate = rospy.get_param('~rotation_rate', -0.3)
        self.runtime = self.moveTime * self._hz
        self.run()

    def left(self):
        self.moveTime = 5.44
        self._forward_rate = rospy.get_param('~forward_rate', 0.0)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.3)
        self.runtime = self.moveTime * self._hz
        self.run()

    def straight(self, num_of_squers):
        self.steps = num_of_squers
        self._forward_rate = rospy.get_param('~forward_rate', 0.1)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.0)
        self.moveTime = 5*self.steps
        self.runtime = self.moveTime * self._hz
        self.run()
    
    def start_serv(self):
        self._forward_rate= rospy.get_param("~forward_rate", 0.1)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.0)
        self.movetime = 40
        self.runtime = self.moveTime * self._hz   
        self.run() 

    def run(self):
        self._angular = 0
        self._linear = 0
        rate = rospy.Rate(self._hz)
        runtime = self.runtime
        while runtime > 0:
            self._set_velocity()
            self._publish()
            rate.sleep()
            runtime -= 1

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        linear = 1.0
        angular = 1.0
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


def main():
    asked_movement = sys.argv
    rospy.init_node('spin')
    app = Move_from_x_to_y(asked_movement[1])



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
