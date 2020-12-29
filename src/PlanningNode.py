#!/usr/bin/env python
 
import roslib; roslib.load_manifest('rosprolog')
from bariago.msg import Cocktail as cocktail_msg
from bariago.srv import CocktailOrder as order
import rospy
from rosprolog_client import PrologException, Prolog

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

class PlanningManager():

    def __init__(self):
        rate = float(1)
        listener_topic = '/cocktail'
        self.publisher_topic = '/prep_order'
        self.kb_topic = '/knowledge_base'

        rospy.loginfo('rate = %d', rate)
        rospy.loginfo('listener topic = %s', listener_topic)
        self.enable = True

        self.prolog = Prolog()
        self.query = None
        # define a subscriber to the cocktail topic
        self.listener = rospy.Subscriber(listener_topic, cocktail_msg, self.cocktail_cb)
        self.nationality = None
        self.taste = None
        # define a timer for the timer callback
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        print("timer called")
    
    def timer_cb(self,  _event):
        self.query = self.prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        for solution in self.query.solutions():
            print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        self.query.finish()

    def cocktail_cb(cocktail_msg):  
        """
        Callback for the cocktail message
        """
        rospy.loginfo('Cocktail msg arrived at plannign node!')
        self.nationality = cocktail_msg.nationality
        self.taste = cocktail_msg.favourite_taste



if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("PlanningManager")
    # Go to class functions that do all the heavy lifting.
    try:
        PlanningManager()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    