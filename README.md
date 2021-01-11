# bariago

Bariago is a bartender robot, that will mix the cocktail you want. The order can be entererd by answering the questions of the order in the terminal. Bariago knows his customers
and knows their preferences. From his knowledge  from previous customers he is able to give recommendations for a fitting cocktail based on the customers preference. To start 
Bariago the following commands must be entered:
1. Open the first terminal in linux:
```
cd catkin_ws/
source devel/setup.bash
roslaunch knowrob_tutorial.launch
```
2. Open second terminal
```
cd catkin_ws/
source devel/setup.bash
roslaunch bariago bariago.launch
```
3. Listen to the text in the terminal and create a order and your cocktail will be served in no time!:)

4. If you want to start a new order:
```
x
NewOrder
```
Note: x can be exhanged for any random input. This is needed to start the callback for a new order.
Now you can create a new order. This process can be repeated.