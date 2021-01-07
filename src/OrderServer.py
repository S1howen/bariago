#!/usr/bin/env python
 
import roslib
from bariago.srv import CocktailOrder as order
import rospy

def order_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a 
    '''
    request = order()
    print('Please answer the following questions:')
    print('Where are you from? \n Type in 1 for Sweden, 2 for German, 3 for Scottland')
    n1 = int(input('Enter a number: '))
    n2 = int(input('Enter another number: '))
    request.nationality = n1
    request.favourite_taste = n2
    return request

rospy.init_node('order_service')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/cocktail_order', order, order_response         # type, and callback
)
rospy.spin()   