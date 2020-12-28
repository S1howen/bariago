#!/usr/bin/env python
 
import roslib; roslib.load_manifest('json_prolog')
from msg import Cocktail as cocktail_msg
from srv import CocktailOrder as order
import rospy
import json_prolog


# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
 
def order_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    return order(
        nationality=1,
        favourite_taste=^
    )

    
class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

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
    # add the ros service
    self.order = order()
    # create the connection to teh service
    self.order_service = rospy.ServiceProxy('/order_service', order)
    self.order = order_service(order_response())
    # Create a publisher for our custom message.
    pub = rospy.Publisher(topic, cocktail_msg)
    # Set the message to publish as our custom message.
    # Initialize message variables.
    self.msg.customer_nationality = 1
    self.msg.favourite_taste = 1
    self.enable = True

    if self.enable:
        self.start()
    else:
        self.stop()

    # Create a timer to go to a callback at a specified interval.
    rospy.Timer(rospy.Duration(1.0 / rate), self.timer_cb)


    def start(self):
        """Turn on publisher."""
        self.pub = rospy.Publisher("example", NodeExampleData, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timer_cb(self, _event):
        """Call at a specified interval to publish message."""
        if not self.enable:
            return
        # Publish our custom message.
        self.pub.publish(self.msg)

    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        # Fill in local variables with values received from dynamic reconfigure
        # clients (typically the GUI).
        self.message = config["message"]
        self.int_a = config["a"]
        self.int_b = config["b"]

        # Check to see if node should be started or stopped.
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]

        # Return the new variables.
        return config

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("CocktailInterface")
    # Go to class functions that do all the heavy lifting.
    try:
        CocktailOrderInterface()
    except rospy.ROSInterruptException:
        pass
    # Allow ROS to go to all callbacks.
    rospy.spin()
        