import sim
import cv2
import numpy as np
from ultralytics import YOLO
import sys

# Load the YOLO model
model = YOLO('best.pt')

# Connect to CoppeliaSim
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID == -1:
    print('Failed to connect to CoppeliaSim')
    sys.exit(1)
else:
    print('Connected to CoppeliaSim')

# Retrieve the vision sensor handle
res, vision_sensor_handle = sim.simxGetObjectHandle(
    clientID, 'Vision_sensor', sim.simx_opmode_blocking)

if res != sim.simx_return_ok:
    print('Failed to retrieve vision sensor handle')
    sys.exit(1)

# Start streaming the vision sensor image
res, resolution, image = sim.simxGetVisionSensorImage(
    clientID, vision_sensor_handle, 0, sim.simx_opmode_streaming)


def get_object_coordinates():
    # Retrieve the vision sensor image
    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, vision_sensor_handle, 0, sim.simx_opmode_buffer)

    if res == sim.simx_return_ok:
        # Convert the image data to a NumPy array
        img = np.array(image, dtype=np.uint8).astype(np.uint8)

        img.resize([resolution[1], resolution[0], 3])

        # Rotate the image by 180 degrees
        img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.flip(img, 1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        cv2.imwrite("image.jpg", img)

        # Process the image using YOLO object detection
        results = model.predict(source=img, show=False, save=False)

        return results
    
def get_image():
    # Retrieve the vision sensor image
    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, vision_sensor_handle, 0, sim.simx_opmode_buffer)

    if res == sim.simx_return_ok:
        # Convert the image data to a NumPy array
        img = np.array(image, dtype=np.uint8).astype(np.uint8)

        img.resize([resolution[1], resolution[0], 3])

        # Rotate the image by 180 degrees
        img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.flip(img, 1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        


