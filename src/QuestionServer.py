#!/usr/bin/env python
 
import roslib 
from bariago.srv import DrinkEvaluation as evaluation_msg
import rospy

def customer_response(request):
    """ 
    Callback function used by the service server to process recommended drink, the bartender wants to prepare for the customer.
    """
    valid_answers = ['y', 'Y', 'n', 'N']
    customer_eval = evaluation_msg()
    customer_eval.recommended_drink = request.recommended_drink
    print('The bartender recommends a ' + request.recommended_drink + '.\n Do you like that drink?')
    n1 = str(raw_input('For yes enter y for no enter n: '))
    
    while n1 not in valid_answers:
        print('Sorry:( Answer not accepted! Please type in something valid!')
        n1 = str(raw_input('For yes enter y for no enter n: '))

    if n1 == 'y' or n1 == 'Y':
        print('Great. Your drink will be served in no time.')
        customer_eval.customer_opinion = True
    elif n1 == 'n' or n1 == 'N':
        print('Okay let me think of something fitting for you!')
        customer_eval.customer_opinion = False
    return customer_eval

rospy.init_node('QuestionServer')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/customer_response', evaluation_msg, customer_response        # type, and callback
)
rospy.spin()   