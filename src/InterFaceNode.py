#!/usr/bin/env python
 
import roslib;
from bariago.msg import Cocktail as cocktail_msg
from bariago.srv import CocktailOrder as order
from bariago.srv import DrinkEvaluation as evaluation_msg
import rospy

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

"""
OPEN TO DO LIST:

- fix the order count to get the right valuee very time
- solve the problem for getting multiple customer orders -> is service the right way?
- 

"""
def order_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a 
    '''
    request = order()
    print('Please answer the following questions:')
    print('Where are you from? \n Type in:\n 1 for Sweden\n 2 for Germany\n 3 for Scottland\n 4 for Spain\n 5 for China\n 6 for England\n 7 for Mexico\n 8 for USA\n 9 for country not listed')
    n1 = int(input('Enter your nationality: '))
    while n1 > 9 and n1 < 1:
        print('Answer not accepted. Please enter a valid number!')
        n1 = int(input('Enter your nationality: '))

    print('What is your favourite taste for a drink?\n Type in:\n 1 for sweet\n 2 for bitter\n 3 for spicy\n 4 for sour\n 5 for salty')
    n2 = int(input('Enter your favourite taste: '))
    while n2 > 5 and n2 < 1:
        print('Answer not accepted. Please enter a valid number!')
        n2 = int(input('Enter your favourite taste: '))

    print('What are you up to? \n Type in:\n 1 for Lets Party!\n 2 drink and chill\n 3 On a date\n 4 for ????\n 5  for mood not listed')
    n3 = int(input('What are you up to: '))
    while n3 > 5 and n1 < 1:
        print('Answer not accepted. Please enter a valid number!')
        n3 = int(input('What are you up to: '))

    print('Do you like hard alcohol?\nType in y for yes or n for No')
    n4 = str(raw_input('Do you like hard alcohol? '))

    valid_answers = ['y', 'Y', 'n', 'N']
    while n4 not in valid_answers:
        print('Answer not accepted. Please enter a valid answer!')
        n4 =str(raw_input('Do you like hard alcohol? '))

    print('answer accepted!')    
    request.nationality = n1
    request.favourite_taste = n2
    if n4 == 'y' or n4 == 'Y':
        request.likes_hard_alcohol = True
    else:
        request.likes_hard_alcohol = False
    request.current_mood = n3
    return request

class CocktailOrderInterface():
# Must have __init__(self) function for a class, similar to a C++ class constructor.

    def __init__(self):
        # Get the ~private namespace parameters from command line or launch file.
        rate = float(1)
        topic = '/cocktail'
        rospy.loginfo('rate = %d', rate)
        rospy.loginfo('topic = %s', topic)
        self.enable = True
        self.msg = cocktail_msg()
        self.order_count = 0
        # add the ros service
        self.order_srv = order()
        # create the connection to the service
        self.order_service = rospy.ServiceProxy('/cocktail_order', order)

        # Create a publisher for our custom message.
        self.pub = rospy.Publisher(topic, cocktail_msg, queue_size=10)
        # Set the message to publish as our custom message.
        
        self.enable = True

        if self.enable:
            self.start()
        else:
            self.stop()

        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        print("timer called")


    def start(self):
        """Turn on publisher."""
        rospy.wait_for_service('/cocktail_order')
        rospy.loginfo('waiting for service done')
        order_msg = order()
        self.order_srv = order_response(order_msg)
        self.msg.customer_nationality = self.order_srv.nationality
        self.msg.favourite_taste = self.order_srv.favourite_taste
        self.msg.customer_number = self.order_count +1
        self.msg.likes_hard_alcohol = self.order_srv.likes_hard_alcohol
        self.msg.current_mood = self.order_srv.current_mood
        self.pub.publish(self.msg)
        rospy.loginfo('current service msg is {}'.format(self.msg))

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, _event):
        """Call at a specified interval to publish message."""
        if not self.enable:
            return
        # Publish our custom message.

        self.pub.publish(self.msg)
        # print('timer cb going')


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("CocktailInterface")
    # Go to class functions that do all the heavy lifting.
    try:
        CocktailOrderInterface()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


        