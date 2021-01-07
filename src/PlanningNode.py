#!/usr/bin/env python
 
import roslib; roslib.load_manifest('rosprolog')
from bariago.srv import DrinkEvaluation as evaluation_msg
from bariago.msg import Cocktail as cocktail_msg
from bariago.msg import CocktailMixing as mixing_msg
import rospy
from rosprolog_client import PrologException, Prolog

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer


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


class CustomerAccount:

    def __init__(self, cocktail_msg):
        self.cocktail_msg = cocktail_msg
        # define  the needed values for a customer
        self.number = cocktail_msg.customer_number
        self.customer_order = cocktail_msg.cocktail_request
        self.nationality = None
        self.favourite_taste = None
        self.customer_name = None
        self.likes_hard_alc = None
        self.current_mood = None
        # store future queries as strings to have the right format
        self.customer_query = None
        self.taste_query = None
        self.mood_query = None
        self.likes_alc_query = None
        self.likes_ordered_cocktail_query = None
        self.customer_drink = None
        self.queries_list = []
        self.d_taste = {}
        self.d_nationality = {}
        self.d_mood = {}
        self.owl_name = "'http://www.chalmers.se/ontologies/ssy235Ontology.owl#"
        self.get_msg_values(cocktail_msg)
    
    def init_dicts(self):
        self.d_taste['1'] = 'Sweet'
        self.d_taste['2'] = 'Bitter'
        self.d_taste['3'] = 'Spicy'
        self.d_taste['4'] = 'Sour'
        self.d_taste['5'] = 'Salty'
        self.d_nationality['1'] = 'Swedish'
        self.d_nationality['2'] = 'German'
        self.d_nationality['3'] = 'Scottish'
        self.d_nationality['4'] = 'Spanish'
        self.d_nationality['5'] = 'Chinese'
        self.d_nationality['6'] = 'Brittish'
        self.d_nationality['7'] = 'Mexican'
        self.d_nationality['8'] = 'American'
        self.d_nationality['9'] = 'Else'
        self.d_mood['1'] = 'LetsParty!'
        self.d_mood['2'] = 'DrinkAndChill'
        self.d_mood['3'] = 'OnDate'
        self.d_mood['4'] = 'Sad'
        self.d_mood['5'] = 'Else'
    

    def get_msg_values(self, cocktail_msg):
        """
        Main method for evaluating the cocktail msg from the InterFaceNode and translate it into the right properties for the customer

        """

        self.customer_drink = cocktail_msg.cocktail_request

        if cocktail_msg.favourite_taste == 1:
            self.favourite_taste = 'Sweet'

        elif cocktail_msg.favourite_taste == 2:
            self.favourite_taste = 'Bitter'
        
        elif cocktail_msg.favourite_taste == 3:
            self.favourite_taste = 'Spicy'

        elif cocktail_msg.favourite_taste == 4:
            self.favourite_taste = 'Sour'
        
        elif cocktail_msg.favourite_taste == 5:
            self.favourite_taste = 'Salty'
      

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
        
        print(self.customer_name)
        print(self.nationality)
            
        # create the nationality and customer number specific strings for the queries:
        # self.customer_query = "rdf_assert("+ self.owl_name+ self.customer_name + "', rdf:type,"+self.owl_name+ self.nationality+ "People')"
        self.customer_query = "rdf_assert("+ self.owl_name+ self.customer_name + "', rdf:type,"+self.owl_name+ "BarCustomer')"
        self.taste_query = "rdf_assert("+ self.owl_name+ self.customer_name + "',"+self.owl_name+ "hasTaste',"+ self.owl_name + str(self.favourite_taste) + "')"
        self.mood_query = "rdf_assert("+ self.owl_name+ self.customer_name + "',"+self.owl_name+ "hasMood',"+ self.owl_name + str(self.current_mood) + "')"
        self.likes_alc_query = "rdf_assert("+ self.owl_name+ self.customer_name + "',"+self.owl_name+ "likesHardAlcohol',"+ self.owl_name + str(self.likes_hard_alc) + "')"
        
        self.likes_ordered_cocktail_query = "rdf_assert("+ self.owl_name+ self.customer_name + "',"+self.owl_name+ "likesDrink',"+ self.owl_name + str(self.customer_drink) + "')"
        self.queries_list.append(self.customer_query)
        self.queries_list.append(self.taste_query)
        self.queries_list.append(self.mood_query)
        self.queries_list.append(self.likes_alc_query)

        # create an instance for the drink the customer has ordered:
        self.customer_drink_query = "rdf_assert("+ self.owl_name+ self.customer_drink +"_"+ str(self.number) + "', rdf:type,"+self.owl_name+ self.customer_drink+"')"
        self.ask_ingredient_query = "rdf_has(" + self.owl_name+ self.customer_drink + "', 'http://www.chalmers.se/ontologies/ssy235Ontology.owl#hasIngredient', O)"
        
        self.queries_list.append(self.customer_drink_query)
        # self.queries_list.append(self.ask_ingredient_query)

        print(self.customer_drink_query)
        # print(self.ask_ingredient_query)

    def return_query_list(self):
        return self.queries_list
    
    def return_ingredient_query(self):
        return self.ask_ingredient_query

class CustomerManager:

    def __init__(self):
        self.customer_list = []
        self.customer_count = 0
        self.query = None
        self.prolog = Prolog()
        
    def update_customer_list(self, cocktail_msg):
        """
        When a new cocktail order arrives a new CustomerAccoung will be crated and the instance will be stored in the customer list
        """
        if cocktail_msg.customer_number > self.customer_count:
            # a new order arrived we need to crate a new instance for our customer
            self.customer_count += 1
            self.customer_list.append(CustomerAccount(cocktail_msg))

        # get all the created queries form the new customer account
        query_list = self.customer_list[-1].return_query_list()
        
        # for further testing service
        query_list.append("owl_individual_of(A, 'http://www.chalmers.se/ontologies/ssy235Ontology.owl#:AlcoholicBeverage')")
        query_list.append("owl_individual_of(A, 'http://www.chalmers.se/ontologies/ssy235Ontology.owl#BarCustomer')")

        # rdfs_individual_of
        # rdfs_subproperty_of(yourOntology:'hasLiquor', P), owl_restriction(S, restriction(P, some_values_from(V))), owl_individual_of(I, yourOntology:'GinTonic').

        # add the new customer instance to prolog by asking all queries
        self.ask_queries(query_list)
        # after the customer instance with its properties is defined and the cocktail instnace was created look for ingredients
        ingredient_query = self.customer_list[-1].return_ingredient_query()
        # Check wich stuff has ingredients!!
        print(ingredient_query)
        self.ask_single_query(ingredient_query)

    def create_customer_recomendation(self):
        pass
    
    def ask_single_query(self, string_query):
        self.query = self.prolog.query(string_query)
        for solution in self.query.solutions():
            if solution != {}:
                if solution['A']:
                    print(solution['A'])
                elif solution['O']:
                    print(solution['O'])
                elif solution['P']:
                    print(solution['P'])
                else:
                    print(solution)
        self.query.finish()
    
    def ask_queries(self, query_list):
        """
        Method to ask all the queries in the query list object and return the answers
        """
        for query in query_list:
            # print(query)
            self.ask_single_query(query)



class PlanningManager():

    def __init__(self):
        rate = float(1)
        listener_topic = '/cocktail'
        self.publisher_topic = '/prep_order'
        self.kb_topic = '/knowledge_base'

        rospy.loginfo('rate = %d', rate)
        rospy.loginfo('listener topic = %s', listener_topic)
        self.enable = True

        self.prolog = Prolog("/rosprolog")
        self.query = None
        # define a subscriber to the cocktail topic
        self.listener = rospy.Subscriber(listener_topic, cocktail_msg, self.cocktail_cb)
        # define a timer for the timer callback
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        print("timer called")
        # crate the customer manager to store and manage all orders of the customers
        self.customer_manager = CustomerManager()
        # crate a client for the QuestionServer to ask the customer if they like the recommended cocktail
        self.question_service = rospy.ServiceProxy('/customer_response', evaluation_msg)
        self.customer_response = evaluation_msg()

    
    def timer_cb(self,  _event):
        # self.query = self.prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        # self.query = self.prolog.query("rdf_assert(ssy235Ontology:'Drink_1', rdf:type, ssy235Ontology:'Beer')")
        # self.query = self.prolog.query("owl_subclass_of(A, ssy235Ontology:'FoodOrDrink')")
        # rdf_assert(ssy235Ontology:'Drink_1', rdf:type, ssy235Ontology:'Beer').
        # rdf_has(Drink_1, ssy235Ontology:'hasTaste', O) 
        # check if there exists and individual of the class
        # owl_individual_of(I,  ssy235Ontology:'Beer').

        # rdf_has(I, ssy235Ontology:'hasTaste', ssy235Ontology:'Beer').
        # rdf_has(ssy235Ontology:'Beer', rdf:type, owl:'Class').
        # self.query = self.prolog.query("rdf_has(I, ssy235Ontology:'hasTaste', ssy235Ontology:'Sweet')")
        # for solution in self.query.solutions():
        #    print(solution)
        #self.query.finish()
        pass

    def cocktail_cb(self, cocktail_msg):  
        """
        Callback for the cocktail message
        """
        # use the customer manager class to create a new instance for a customer account
        if cocktail_msg.customer_number > self.customer_manager.customer_count:
            print('new order arrived!')
            self.customer_manager.update_customer_list(cocktail_msg)
        # self.customer_manager
        # test the new query
        
        
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("PlanningManager")
    # Go to class functions that do all the heavy lifting.
    try:
        PlanningManager()
        rospy.spin()

    except rospy.ROSInterruptException:
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

<node name="rosprolog" pkg="rosprolog" type="rosprolog_node" output="screen"/>

        query = self.prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        for solution in query.solutions():
            print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        query.finish()
    

        self.query = self.prolog.query(self.customer_list[self.customer_count-1].customer_query)

        for solution in self.query.solutions():
            print(solution)
        self.query.finish()

        self.query = None

        self.query = self.prolog.query( self.customer_list[self.customer_count-1].taste_query)

        for solution in self.query.solutions():
            print(solution)
        self.query.finish()

        self.query = self.prolog.query( "rdf_assert('http://www.chalmers.se/ontologies/"
        "ssy235Ontology.owl#Drink_1', rdf:type, "
        "'http://www.chalmers.se/ontologies/ssy235Ontology.owl#Beer')")
        for solution in self.query.solutions():
            print(solution)
        self.query.finish()

        self.query = self.prolog.query("owl_individual_of(I, "
        "'http://www.chalmers.se/ontologies/ssy235Ontology.owl#Beer')")

        for solution in self.query.solutions():
            # print(solution)
            print(solution['I'])
        self.query.finish()
"""