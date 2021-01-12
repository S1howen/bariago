# Bariago

Bariago is a bartender robot, that will mix and serve cocktails from an terminal order. The order can be entererd by answering the questions of bariago in the terminal. Bariago knows his customers and their preferences. From his build up knowledge from previous customers, he is able to give recommendations for a fitting cocktail based on the customers choices. To start 
Bariago the following commands must be entered:

1. Open the first terminal in linux:
```
cd catkin_ws/
source devel/setup.bash
roslaunch knowrob_tutorial onto_robot.launch
```

2. Open a second terminal
```
cd catkin_ws/
source devel/setup.bash
roslaunch bariago bar_env.launch
```
3. Open a third terminal
```
cd catkin_ws/
source devel/setup.bash
roslaunch bariago bariago.launch
```
3. Listen to the text in the terminal and create a order and your cocktail will be served in no time!:)

4. If you want to start a new order type in the second terminal:
```
x
NewOrder
```
Note: x can be exhanged for any random input. This is needed to start the callback for a new order.
Now you can create a new order. This process can be repeated.
