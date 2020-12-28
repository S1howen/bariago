#!/usr/bin/env python
 
import roslib; 
from srv import CocktailOrder as order
import rospy
import json_prolog

def order_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    return order(
        nationality=1,
        favourite_taste=1
    )

rospy.init_node('order_service')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/cocktail_order', order, order_response         # type, and callback
)
rospy.spin()   