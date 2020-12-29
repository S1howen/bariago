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
        # self.query = self.prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        self.query = self.prolog.query("rdf_has(ssy235Ontology:'FoodOrDrink', rdf:type, O)")
        # self.query = self.prolog.query("owl_subclass_of(A, ssy235Ontology:'FoodOrDrink')")
        # rdf_assert(ssy235Ontology:'Drink_1', rdf:type, ssy235Ontology:'Beer').
        # rdf_has(Drink_1, ssy235Ontology:'hasTaste', O)
        # check if there exists and individual of the class
        # owl_individual_of(I,  ssy235Ontology:'Beer').

        # rdf_has(I, ssy235Ontology:'hasTaste', ssy235Ontology:'Beer').
        # rdf_has(ssy235Ontology:'Beer', rdf:type, owl:'Class').
        self.query = self.prolog.query("rdf_has(I, ssy235Ontology:'hasTaste', ssy235Ontology:'Sweet')")
        for solution in self.query.solutions():
            print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        self.query.finish()

    def cocktail_cb(self, cocktail_msg):  
        """
        Callback for the cocktail message
        """
        # print('Cocktail msg arrived at plannign node!')
        if cocktail_msg.favourite_taste == 1:
            self.taste = 'Sweet'
        elif cocktail_msg.favourite_taste == 2:
            self.taste = 'Bitter'
        elif cocktail_msg.favourite_taste == 3:
            self.taste = 'Spicy'
        elif cocktail_msg.favourite_taste == 4:
            self.taste = 'Sour'
        elif cocktail_msg.favourite_taste == 5:
            self.taste = 'Salty'

        if cocktail_msg.customer_nationality == 1:
            self.nationality = 'Swedish'
            customer_query = "rdf_assert(ssy235Ontology:'Sweden_1', rdf:type, ssy235Ontology:'SwedishPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Sweden_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 2:
            self.nationality = 'German'
            customer_query = "rdf_assert(ssy235Ontology:'German_1', rdf:type, ssy235Ontology:'GermanPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'German_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 3:
            self.nationality = 'Scottish'
            customer_query = "rdf_assert(ssy235Ontology:'Scottish_1', rdf:type, ssy235Ontology:'ScottishPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Scottish_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 4:
            self.nationality = 'Spanish'
            customer_query = "rdf_assert(ssy235Ontology:'Spanish_1', rdf:type, ssy235Ontology:'SpanishPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Sweden_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 5:
            self.nationality = 'Chinese'
            customer_query = "rdf_assert(ssy235Ontology:'Chinese_1', rdf:type, ssy235Ontology:'ChinesePeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Spanish_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 6:
            self.nationality = 'Brittish'
            customer_query = "rdf_assert(ssy235Ontology:'Brittish_1', rdf:type, ssy235Ontology:'BrittishPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Brittish_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 7:
            self.nationality = 'Mexican'
            customer_query = "rdf_assert(ssy235Ontology:'Mexican_1', rdf:type, ssy235Ontology:'MexicanPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'Mexican_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 8:
            self.nationality = 'American'
            customer_query = "rdf_assert(ssy235Ontology:'American_1', rdf:type, ssy235Ontology:'AmericanPeople')"
            taste_query = "rdf_assert(ssy235Ontology:'American_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        elif cocktail_msg.customer_nationality == 9:
            self.nationality = 'Else'
            customer_query = "rdf_assert(ssy235Ontology:'People_1', rdf:type, ssy235Ontology:'People')"
            taste_query = "rdf_assert(ssy235Ontology:'People_1', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("PlanningManager")
    # Go to class functions that do all the heavy lifting.
    try:
        PlanningManager()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

class CustomerAccount:

    def __init__(self, cocktail_msg):
        self.cocktail_msg = cocktail_msg
        # define  the needed values for a customer
        self.number = cocktail_msg.customer_number
        self.nationality = None
        self.favourite_taste = None
        self.customer_name = None
        # store future queries as strings to have the right format
        self.customer_query = None
        self.taste_query = None
        self.get_msg_values()

    def get_msg_values(self):
        """
        Main method for evaluating the cocktail msg from the InterFaceNode and translate it into the right properties for the customer

        """
        if cocktail_msg.customer_nationality == 1:
            self.customer_name = 'Swedish_customer'+ str(self.number) 
            self.nationality = 'Swedish'

        elif cocktail_msg.customer_nationality == 2:
            self.customer_name = 'German_customer'+ str(self.number) 
            self.nationality = 'German'


        elif cocktail_msg.customer_nationality == 3:
            self.customer_name = 'Scottish_customer'+ str(self.number) 
            self.nationality = 'Scottish'


        elif cocktail_msg.customer_nationality == 4:
            self.customer_name = 'Spanish_customer'+ str(self.number) 
            self.nationality = 'Spanish'


        elif cocktail_msg.customer_nationality == 5:
            self.customer_name = 'Chinese_customer'+ str(self.number) 
            self.nationality = 'Chinese'


        elif cocktail_msg.customer_nationality == 6:
            self.customer_name = 'Brittish_customer'+ str(self.number) 
            self.nationality = 'Brittish'


        elif cocktail_msg.customer_nationality == 7:
            self.customer_name = 'Mexican_customer'+ str(self.number) 
            self.nationality = 'Mexican'

        elif cocktail_msg.customer_nationality == 8:
            self.customer_name = 'American_customer'+ str(self.number) 
            self.nationality = 'American'


        elif cocktail_msg.customer_nationality == 9:
            self.customer_name = 'customer'+ str(self.number) 
            self.nationality = 'Else'
            

        # create the nationality and customer nubmer specific strings for the queries:
        self.customer_query = "rdf_assert(ssy235Ontology:'"+ self.customer_name+ "', rdf:type, ssy235Ontology:'"+self.nationality+ "People')"
        self.taste_query = "rdf_assert(ssy235Ontology:'"+ self.customer_name+ "', ssy235Ontology:'hasTaste', ssy235Ontology:'" + str(self.taste) + "')"
        print(self.customer_query)
        print(self.taste_query)


class CustomerManager:

    def __init__(self):
        self.customer_list = []
        self.customer_count = 0
        
    def update_customer_list(cocktail_msg):
        """
        When a new cocktail order arrives a new CustomerAccoung will be crated and the instance will be stored in the customer list
        """
        if cocktail_msg.customer_count > self.customer_count:
            # a new order arrived we need to crate a new instance for our customer
            self.customer_count += 1
            self.customer_list.append(CustomerAccount(cocktail_msg))

    def create_customer_recomendation(self):
        pass
    



"""
ask for a object type rdf:type
ask for a object property: rdf_has(I, ssy235Ontology:'objectProperty', O)
crate an instance of an object rdf_assert()

rdf_has(O, ssy235Ontology:'hasTaste', Drink_1).
rdf_has(O, ssy235Ontology:'hasTaste', ssy235Ontology:'Beer').
rdf_has(ssy235Ontology:'FoodOrDrink', ssy235Ontology:'hasTaste', O).    a
rdf_has(ssy235Ontology:'hasTaste', rdf:type, O).
rdf_has(ssy235Ontology:'Beer',rdf:'resource', O).
rdf_has(I,rdf:'resource', ssy235Ontology:'Beer').
Create an instance of  the beer class

rdf_assert(ssy235Ontology:'Drink_1', rdf:type, ssy235Ontology:'Beer').
rdf_has(ssy235Ontology:'Drink_1', ssy235Ontology:'hasTaste', O).
rdf_has(ssy235Ontology:'Beer', ssy235Ontology:'hasTaste', O).
assert and object property

rdf_assert(ssy235Ontology:'Drink_1', ssy235Ontology:'hasTaste', ssy235Ontology:'Bitter').

rdf_has(ssy235Ontology:'BrittishPeople', ssy235Ontology:'likesDrink', O).

rdf_assert(ssy235Ontology:'German_1', rdf:type, ssy235Ontology:'GermanPeople')
"""