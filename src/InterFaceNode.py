#!/usr/bin/env python
import time
import roslib
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
    cocktail_d = {}
    cocktail_d['GinTonic'] = 'GinTonic'
    cocktail_d['Gin Tonic'] = 'GinTonic'
    cocktail_d['Gintonic'] = 'GinTonic'
    cocktail_d['gintonic'] = 'GinTonic'
    cocktail_d['CubaLibre'] = 'CubaLibre'
    cocktail_d['cuba libre'] = 'CubaLibre'
    cocktail_d['Cubalibre'] = 'CubaLibre'
    cocktail_d['Cuba Libre'] = 'CubaLibre'
    cocktail_d['WhiskeyCola'] = 'WhiskeyCola'
    cocktail_d['whiskeyCola'] = 'WhiskeyCola'
    cocktail_d['Whiskey Cola'] = 'WhiskeyCola'
    cocktail_d['whiskey cola'] = 'WhiskeyCola'
    cocktail_d['WhiskeySour'] = 'WhiskeySour'
    cocktail_d['whiskeysour'] = 'WhiskeySour'
    cocktail_d['Whiskey Sour'] = 'WhiskeySour'
    cocktail_d['whiskey sour'] = 'WhiskeySour'
    cocktail_d['Beer'] = 'Beer'
    cocktail_d['beer'] = 'Beer'
    cocktail_d['Wine'] = 'Wine'
    cocktail_d['wine'] = 'Wine'
    cocktail_d['SurpriseMe'] = 'SurpriseMe'
    cocktail_d['SurpriseMe!'] = 'SurpriseMe'
    cocktail_d['Surprise Me'] = 'SurpriseMe'
    cocktail_d['surprise me'] = 'SurpriseMe'
    cocktail_d['Surprise me'] = 'SurpriseMe'

    request = order()
    print('Welcome, I am Bariago.\nHow can I help you? I see you are thirsty. What dou you want to order?')
    print('Currently we offer the following drinks:')
    print('Beer, Wine and the Cocktails: GinTonic, WhiskeyCola, CubaLibre, WhiskeySour\n If you want to get a recommendation from the barkeeper based on your preferences type: SurpriseMe')
    n_order = str(raw_input('Please enter your order: '))
    while n_order not in cocktail_d:
        n_order = str(raw_input('There seems to be a mistake:(\nRepeat your order please: '))
    
    print('Of course. Drink will be ready in no time!')
    print('I see you are new at this place.')
    print('Where are you from? \n Type in:\n 1 for Sweden\n 2 for Germany\n 3 for Scottland\n 4 for Spain\n 5 for China\n 6 for England\n 7 for Mexico\n 8 for USA\n 9 for country not listed')
    n1 = int(input('Enter your nationality: '))
    while n1 > 9 and n1 < 1:
        print('Answer not accepted. Please enter a valid number!')
        n1 = int(input('Enter your nationality: '))

    print('What is your favourite taste for a drink?\n Type in:\n 1 for sweet\n 2 for bitter\n 3 for sour')
    n2 = int(input('Enter your favourite taste: '))
    while n2 > 5 and n2 < 1:
        print('Answer not accepted. Please enter a valid number!')
        n2 = int(input('Enter your favourite taste: '))

    print('What are you up to? \n Type in:\n 1 for Lets Party!\n 2 drink and chill\n 3 On a date\n 4 for Sad\n 5  for mood not listed')
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
    cocktail_order = cocktail_d[n_order]  
    request.cocktail_request = cocktail_order
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
        # rospy.loginfo('rate = %d', rate)
        # rospy.loginfo('topic = %s', topic)
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
        if self.order_count == 0:
            time.sleep(3)
        self.get_new_order()
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        print('If you want to start a new order please type in NewOrder')
        self.continue_1 = True
        self.run_node()

    def run_node(self):
        
        while self.continue_1:
            self.start_new_order()

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, _event):

        # Publish our custom message.
        self.pub.publish(self.msg)
        # print('timer cb going')
        # self.start_new_order()

    def get_new_order(self):
        self.order_count  = self.order_count+ 1
        if self.order_count == 0:
            time.sleep(3)
        rospy.wait_for_service('/cocktail_order')
        rospy.loginfo('waiting for service done')
        order_msg = order()
        self.order_srv = order_response(order_msg)
        self.msg.customer_nationality = self.order_srv.nationality
        self.msg.favourite_taste = self.order_srv.favourite_taste
        self.msg.customer_number = self.order_count 
        self.msg.likes_hard_alcohol = self.order_srv.likes_hard_alcohol
        self.msg.current_mood = self.order_srv.current_mood
        self.msg.cocktail_request = self.order_srv.cocktail_request
        # publish the new order
        # Update the customer_number
        

    def start_new_order(self):
        dict_order = {}
        dict_order['NewOrder'] = 'NewOrder'
        dict_order['Neworder'] = 'NewOrder'
        dict_order['New Order'] = 'NewOrder'
        dict_order['new order'] = 'NewOrder'
        dict_order['neworder'] = 'NewOrder'
        n1 = str(raw_input())

        while True:
            try:
                n1 = str(raw_input('Please Type in NewOrder if you want to start a new order:'))
                if n1 in dict_order:
                    break
                print("Invalid Input entered")
            except Exception as e:
                print(e)

        start_new_order = dict_order[n1]
        if start_new_order == 'NewOrder':
            self.get_new_order()

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


        