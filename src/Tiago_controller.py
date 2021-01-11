#! /usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from actionlib import SimpleActionClient, SimpleGoalState
import trajectory_msgs.msg
import roslib
import math
import sys
import time
from geometry_msgs.msg import Twist
from bariago.msg import 


ARM_JOINTS= ["arm_1_joint", "arm_2_joint", "arm_3_joint","arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

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
        elif asked_movement == "table":
            self.straight(0.8)
        elif asked_movement == "btable":
            self.straight(-0.8)
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
            self.straight(6)
            time.sleep(1)
            self.right()
        elif asked_movement == "lime_serv":
            self.right()
            time.sleep(1)
            self.straight(8)
            time.sleep(1)
            self.right()
            self.straight(self.start_serv)  
        #self._backward_rate = rospy.get_param('~backward_rate', 0.3)
    def right(self):
        self.moveTime = 5.13
        self._forward_rate = rospy.get_param('~forward_rate', 0.0)
        self._rotation_rate = rospy.get_param('~rotation_rate', -0.3)
        self.runtime = self.moveTime * self._hz
        self.run()

    def left(self):
        self.moveTime = 5.06
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


def arm_init():
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)


    point = JointTrajectoryPoint()
    point.positions = [0.25, -0.38,-1.5,1.49,0.0,0.2,-0.35]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(6.0)
    trajectory.points.append(point)

    return trajectory

def arm_down_up():
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = [0.25,-0.3,-1.7,1.49,0.0,0.0,-0.35]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)


    point = JointTrajectoryPoint()
    point.positions = [0.25,-0.2,-2.2,1.49,0.0,-0.6,-0.3]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(6.0)
    trajectory.points.append(point)

    return trajectory

def arm_up():
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = [0.25,-0.2,-2.2,1.49,0.0,-0.6,-0.3]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)
    return trajectory

def arm_down():
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = [0.25,-0.3,-1.7,1.49,0.0,0.0,-0.35]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)
    return trajectory

def arm_serv():
    trajectory = JointTrajectory()
    trajectory.joint_names = ARM_JOINTS

    point = JointTrajectoryPoint()
    point.positions = [0.25,-0.38,-1.5,1.49,0.0,0.2,-0.35]
    point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    point.time_from_start = rospy.Duration(3.0)
    trajectory.points.append(point)
    return trajectory
 
class ArmController:
    def __init__(self):
        arm = "arm"
        self.client = SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        #wait for the action servers to come up 
        iteration = 0
        max_iter = 3
        while not self.client.wait_for_server(rospy.Duration(2.0)) and max_iter> iteration:
            iteration += 1

    def start_trajectory(self, trajectory, set_time_stamp=True, wait=True):
        """Creates an action from the trajectory and sends it to the server"""
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        if set_time_stamp:
            goal.trajectory.header.stamp = rospy.Time.now()      
        self.client.send_goal(goal)

        if wait:
            self.wait()
 
    def wait(self):
        self.client.wait_for_result()

    def is_done(self):
        return self.client.get_state() > SimpleGoalState.ACTIVE

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

class Movement_manager:
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.arm = ArmController()
        self.app = None
        self.cocktail_dict = {}
        self.cocktail_dict['CubaLibre'] = ["arm_init",'base_table',"arm_up","base_start_rum","arm_down_up","base_rum_cola","arm_down_up","base_cola_serv","arm_serv"]
        self.cocktail_dict["GinTonic"] = ["arm_init",'base_table',"arm_up","base_start_gin","arm_down_up","base_gin_tonic","arm_down_up","base_tonic_serv","arm_serv"]
        self.cocktail_dict["WhiskeyCola"] = ["arm_init",'base_table',"arm_up","base_start_wis","arm_down_up","base_wis_cola","arm_down_up","base_cola_serv","arm_serv"]
        self.cocktail_dict["WhiskeySour"] = ["arm_init",'base_table',"arm_up","base_start_wis","arm_down_up","base_wis_lime","arm_down_up","base_lime_serv","arm_serv"]
        self.cocktail_dict["Beer"] = ["arm_init",'base_table',"arm_up","base_start_beer","arm_down_up","base_beer_serv","arm_serv"]
        self.cocktail_dict["Wine"] = ["arm_init",'base_table',"arm_up","base_start_wine","arm_down_up","base_wine_serv","arm_serv"]
        self.function_mapping = {"arm_init": arm_init(), "arm_down": arm_down(), "arm_serv": arm_serv(), "arm_up": arm_up(), "arm_down_up": arm_down_up()}
        self.topic_subscriber = '/prep_order'
        rate = float(1) # needed for a callback loop how often in a second the callback will be called

	    # create a subscriber
	    #self.subscriber = rospy.Subscriber("/prep_order", messagevariable, subscriber_callback_function)
	
	    # create a publisher
        self.pub = rospy.Publisher(self.topic_subscriber, message_type, queue_size=1)
	
	    # define a timer for a timer callback
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)

    def subscriber_callback_function(self, messagevariable):
	    # thsi gets called every time a message arrives use the messgae to store the stuff in your variables and calciclate somethjing
        cocktail_name = messge.cocktail_name
        list_with_movements =  self.cocktail_dict[cocktail_name]
        for movement in list_with_movements:
            if 'arm' in movement:
                self.arm.start_trajectory(movement[4:])
            elif 'base' in movement:
                self.app = Move_from_x_to_y(movement[5:])
        print('Here is your drink. Enjoy!')

	
    def timer_cb(self, _event):
        #"""Call at a specified interval to publish message or do stuff every loop"""
        # Publish our custom message.
        self.pub.publish(msg)
        # print('timer cb going')
# define all the class parameters here
    # you need to define the topics you want to sucbscribe and publuish here


    def test_function(self):
        cocktail_name = "WhiskeySour"
        list_witea_movements = self.cocktail_dict[cocktail_name]
        for movement in list_witea_movements:
            if "arm" in movement:
                rospy.loginfo(movement)
                self.arm.start_trajectory(self.function_mapping[movement])
                time.sleep(2)
            elif 'base' in movement:
                rospy.loginfo(movement)
                self.app = Move_from_x_to_y(movement[5:])
                time.sleep(2)
        print('Here is your drink. Enjoy!')
        


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("Tiago_mover")
    # Go to class functions that do all the heavy lifting.
    try:
        #Movement_manager()
        Movement_manager().test_function()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass

