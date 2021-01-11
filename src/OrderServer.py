#!/usr/bin/env python
 
import roslib
from bariago.srv import CocktailOrder as order
import rospy

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
    cocktail_d['WhiskyCola'] = 'WhiskyCola'
    cocktail_d['whiskeyCola'] = 'WhiskeyCola'
    cocktail_d['Whiskey Cola'] = 'WhiskeyCola'
    cocktail_d['whiskey cola'] = 'WhiskeyCola'
    cocktail_d['WhiskySour'] = 'WhiskyCola'
    cocktail_d['whiskeysour'] = 'WhiskeyCola'
    cocktail_d['Whiskey Sour'] = 'WhiskeyCola'
    cocktail_d['whiskey sour'] = 'WhiskeyCola'
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
    print('Welcome, I am Bariago.\n How can I help you?\n I see you are thirsty. What dou you want to order?')
    print('Currently we offer the following drinks:')
    print('Beer, Wine, \nCocktails: GinTonic, WhiskeyCola, CubaLibre, WhiskeySour\n If you want to get a recommendation from the barkeeper based on your preferences type: SurpriseMe')
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

rospy.init_node('order_service')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/cocktail_order', order, order_response         # type, and callback
)
rospy.spin()   