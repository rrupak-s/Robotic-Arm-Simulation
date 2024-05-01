import sys
# import torch
from library import nlp_command as nc
from library import arm_lib as al
from library import problem_lib as pl
from library import vision_lib as vl
from library import transform_lib as tl
from library import kine_lib2 as kl
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import pygame
import tempfile
import cv2
import time
# copelia sim here-----------------------------
client = RemoteAPIClient()

# all simIK.* type of functions and constants
simIK = client.getObject('simIK')
sim = client.getObject('sim')
simBase = sim.getObject('/base_dyn')
simTip = sim.getObject('/base_dyn/tip')
# simTarget = sim.getObject('/base_dyn/target')
# sphere = sim.getObject('/base_dyn/manipSphere')
g_joint1 = sim.getObjectHandle('/base_dyn/gripper_joint1')
g_joint2 = sim.getObjectHandle('/base_dyn/gripper_joint2')
cup_0 = sim.getObjectHandle('/Cup[0]/visible_transparent')
cup_1 = sim.getObjectHandle('/Cup[1]/visible_transparent')
joint1 = sim.getObjectHandle('/base_dyn/joint1')
joint2 = sim.getObjectHandle('/base_dyn/joint2')
joint3 = sim.getObjectHandle('/base_dyn/joint3')
joint4 = sim.getObjectHandle('/base_dyn/joint4')

# to find the homegeneous matrix
simBase = sim.getObject('/base_dyn')
# simTip = sim.getObject('/base_dyn/tip')

sim.setJointTargetPosition(joint1, 0)
sim.setJointTargetPosition(joint2, 0)
sim.setJointTargetPosition(joint3, 0)
sim.setJointTargetPosition(joint4, np.deg2rad(0))

time.sleep(1)

# sys.path.append(r"C:\Users\rupak\Desktop\final_sqee\openspeech_cli")
# sys.path.append(r"C:\Users\rupak\Desktop\final_sqee")
# from run_this import voice_command
# words_to_check = ["sprite", "water", "coffee", "orange"]

region_corners = [(0.825, 0.250), (0.825, -0.300),
                  (0.300, -0.300), (0.300, 0.250)]
# Initialize a dictionary to store camera coordinates for each class
class_coordinates = {}
cup_no = 0

initial_guess = np.array([0.0, 0.0, 0.0, 0.0])

# Main loop to continuously process the camera image
while True:

    pygame.init()

    coor_cup = []
    coor_obstacle = []
    coor_dispenser = []
    command_label = -1
    # Check for command
    print("Enter your command (1 to continue, 0 to exit): ")
    command = input()

    # Convert command to integer to perform comparison
    command = int(command)

    if command == 0:  # Exit the loop if 0 is entered
        break
    elif command == 1:

        results = vl.get_object_coordinates()

        # Clear the dictionary at the beginning of each frame
        class_coordinates = {}

        # Process the detected objects
        for result in results:
            for box in result.boxes:
                class_id = box.cls[0].item()
                class_label = result.names[class_id]
                # Get the normalized bounding box coordinates
                x_center, y_center, norm_width, norm_height = box.xywh[0]

                # Store the camera coordinates in the dictionary
                if class_label not in class_coordinates:
                    class_coordinates[class_label] = []

                class_coordinates[class_label].append((x_center, y_center))

        target_class_label = "0"
        if target_class_label in class_coordinates:
            coor_cup = class_coordinates[target_class_label]

        target_class_label = "1"
        if target_class_label in class_coordinates:
            coor_dispenser = class_coordinates[target_class_label]

        target_class_label = "4"
        if target_class_label in class_coordinates:
            coor_obstacle = class_coordinates[target_class_label]
        print(coor_cup)

        coor_cup = tl.transform_coordinates(coor_cup)
        print(coor_cup)
        coor_obstacle = tl.transform_coordinates(coor_obstacle)
        # print(coor_obstacle)
        if coor_cup != []:
            to_pick = pl.pick_glass(coor_cup)
            x_cup, y_cup = to_pick

            # print(x_cup, y_cup)
        problem_count = pl.vision_problem_check(coor_cup)

        blockage = pl.block_check(coor_obstacle, region_corners)
        # print(blockage)

        if len(coor_dispenser) == 0:
            print("vision_obstructed")
            # Something is blocking the vision sensor. Could you please clear it
            # Load and play the audio file
            pygame.mixer.music.load(
                r'C:\Users\rupak\Desktop\vrep\feedback_aud\obstruction.mp3')
            pygame.mixer.music.play()

            # Wait until the audio finishes playing
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            # Quit pygame
            pygame.quit()
        else:
            if problem_count == 0:
                if len(blockage) != 0:
                    for block in blockage:
                        print(block)
                        if block == True:
                            print("something is blocking the dispensor")
                            pygame.mixer.music.load(
                                r'C:\Users\rupak\Desktop\vrep\feedback_aud\blocking dispenser.mp3')
                            pygame.mixer.music.play()
                            b = 0
                            break
                        else:
                            b = 1
                elif len(blockage) == 0 or b == 1:
                    print("no problems")

                    # pygame.mixer.music.load(
                    #     r'C:\Users\rupak\Desktop\final_sqee\datasets\LibriSpeech\dev-test\12\212122\1.flac')
                    # pygame.mixer.music.play()
                    # v_command = voice_command()
                    # command_label = nc.check_sentence(
                    #     v_command, words_to_check)
                    # print("command is ", command_label)

                    # if (command_label == -1):
                    #     print("comm erro")
                    # elif (command_label == 0):
                    #     print("ordered drink is sprite")
                    # elif (command_label == 1):
                    #     print("ordered drink is water")
                    # elif (command_label == 2):
                    #     print("ordered drink is coffee")
                    # elif (command_label == 3):
                    #     print("ordered drink is orange")

                    desired_position = np.array([x_cup, y_cup, 0])

                    al.rel_cup()
                    initial_guess = al.move_arm(
                        initial_guess, desired_position)

                    al.grab_cup()
                    vl.get_object_coordinates()
                    print(initial_guess)
                    initial_guess = al.to_disp(initial_guess)
                    sim.setShapeColor(
                        cup_1, None, sim.colorcomponent_ambient_diffuse, [0, 0, 0])

                    # Step 3: Apply the changes
                    # sim.reApply()
                    
                    print(initial_guess)
                    initial_guess = al.to_client(initial_guess)
                    
                    al.rel_cup()
                    
                    sim.setJointTargetPosition(joint1, 0)
                    sim.setJointTargetPosition(joint2, 0)
                    sim.setJointTargetPosition(joint3, 0)
                    sim.setJointTargetPosition(joint4, np.deg2rad(0))

                    # al.mov_tocup(x_cup, y_cup)

                    # al.mov_todisp()

                    # al.fill_cup(cup_no, command_label)

                    # al.mov_toserve(cup_no)

                    # al.mov_todef()

                    # cup_no = cup_no + 1

            elif problem_count == 1:
                print("glasses are unreachable")
                # Load and play the audio file
                pygame.mixer.music.load(
                    r'C:\Users\rupak\Desktop\vrep\feedback_aud\unreachable.mp3')
                pygame.mixer.music.play()

                # Wait until the audio finishes playing
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)

                # Quit pygame
                pygame.quit()

            elif problem_count == 2:
                print("no glass available")
                # The drinking glasses are currently unavailable. Kindly restock them
                pygame.mixer.music.load(
                    r'C:\Users\rupak\Desktop\vrep\feedback_aud\noglass.mp3')
                pygame.mixer.music.play()

                # Wait until the audio finishes playing
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)

                # Quit pygame
                pygame.quit()

        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit the loop

            break
    else:
        print('Failed to retrieve vision sensor image')

# Disconnect from CoppeliaSim
print('Connection closed')
