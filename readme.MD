# Automatic Drink Serving Robotic Arm

The robotic arm in this simulation can take a user command with speech recognition model. According to the command it searchs for the empty glass in the workspace using object detection model. dispenses the drink according to the user command and finally serves it to the client.

## Major Funcationalities
1. Operates on the voice command from the user
2. Initializes only with wake word "HEY ARM"
3. A working command can be "HEY ARM PROVIDE ME WITH A CUP OF COFFEE"
4. Varieties of drinks : coffee, water, sprite, orange juice
5. Object detection model to detect empty glasses
6. Dispenses the drink ordered by client
7. Serves filled glass to the client 

## How to use.
1. Install coppelia EDU (version 4.5.1 recommended)
2. Install python dependencies included in requirements
3. Run python script (main.py) to operate the robotic arm
