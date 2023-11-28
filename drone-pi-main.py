# Import packages
import os
import argparse
import cv2
import numpy as np
import sys
import time
import serial
import keyboard
from threading import Thread
from tflite_runtime.interpreter import Interpreter
from tflite_runtime.interpreter import load_delegate

#################################################################
# WHERE TO RUN THE CODE 
#################################################################
# Run the code from the same directory where the model folder exists

#################################################################
# IMPORTANT 
#################################################################
# 1. Camera
    # This code will only work on non-Rasbpian OS (eg. Ubuntu 20.04), on Rasbpian, use picamera2
    # Detect camera in Ubuntu 20.04:
    # $ sudo usermod -aG video <username>
    # $ sudo nano /boot/firmware/config.txt
    # Add the following: start_x=1
    # save and reboot
# 2. Coral USB Accelerator
    # pip install tflite-runtime==2.9.1  (2.9.1 is a MUST)
    # $ sudo apt install libedgetpu1-std or sudo apt install libedgetpu1-max (ONLY 1 SHOULD BE THERE, DEPENDING ON THE DESIRED PERFORMANCE)
    # Python 3.6-3.9 or less is required (TESTED 3.8 ONLY) 


class VideoStream:
    """Camera object that controls video streaming from the Picamera"""
    def __init__(self,resolution=(640,480),framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(cv2.CAP_PROP_FRAME_WIDTH,resolution[0])
        ret = self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT,resolution[1])
        ret = self.stream.set(cv2.CAP_PROP_FPS, framerate)
            
        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

	# Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
	# Start the thread that reads frames from the video stream
        Thread(target=self.update,args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True

# Define colors
red = (0, 0, 255)
yellow = (0, 255, 255)
green = (0, 255, 0)
white = (255, 255, 255)
black = (0, 0, 0)
cyan = (255, 255, 0)

# Define window size
WINDOW_WIDTH = 600
WINDOW_HEIGHT = 600

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--coral', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true', default=False)
parser.add_argument('--debug', help='Enable debugging eg. show camera window, draw boxes, show fps, etc',
                    action='store_true', default=False)
parser.add_argument('--pics', help='Enable taking pictures mode. Pics will get saved in recordings folder',
                    action='store_true', default=False)
args = parser.parse_args()

min_conf_threshold = 0.35
use_TPU = args.coral
debug = args.debug
pics = args.pics

if debug:
    print(f"Using Coral: {use_TPU}")
    print(f"Debugging: {debug}")
    print(f"Taking Pics: {pics}")

# Get path to current working directory
CWD_PATH = os.getcwd()      

# Path to .tflite file, which contains the model that is used for object detection
# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    PATH_TO_GRAPH = os.path.join(CWD_PATH, 'models', 'custom_model_lite_latest', 'edgetpu.tflite')
else:
    PATH_TO_GRAPH = os.path.join(CWD_PATH, 'models', 'custom_model_lite_latest', 'detect_quant.tflite')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'models', 'custom_model_lite_latest', 'labelmap.txt')

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]


# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_GRAPH,
                              experimental_delegates=[load_delegate('libedgetpu.so.1')])
else:
    interpreter = Interpreter(model_path=PATH_TO_GRAPH)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]


# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0

# Initialize HC-05 Bluetooth module and bind r
os.system('sudo rfcomm bind rfcomm0 00:14:03:05:01:C5')
BTserialChannel = serial.Serial("/dev/rfcomm0", baudrate=115200)

# Initialize video stream
videostream = VideoStream(resolution=(width, height),framerate=30).start()

time.sleep(1)
if debug:
    cv2.namedWindow('Object Detector', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Object Detector', WINDOW_WIDTH, WINDOW_HEIGHT)

# Initialize detection flags to check every frame
red_flag = False
yellow_flag = False
green_flag = False
robot_flag = False

if pics:
    flight_folder_counter = 0
    pics_counter = 0
    if not os.path.exists('recordings'):
        print("NO /recordings folder. Creating one...")
        os.makedirs('recordings')
    else:
        print('/recordings folder already exists. No action.')

    while True:
        flight_folder = f'flight{flight_folder_counter:03d}'
        if not os.path.exists(os.path.join('recordings', flight_folder)):
            print(f"Creating {flight_folder} folder...")
            os.makedirs(os.path.join('recordings', flight_folder))
            break
        else:
            flight_folder_counter = flight_folder_counter + 1



# For frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
while True:

    # Start timer (for calculating frame rate) if debug
    if debug or pics:
        timer = cv2.getTickCount()

    # Reset flags every frame
    red_flag = False
    yellow_flag = False
    green_flag = False
    robot_flag = False

    frame1 = videostream.read()

    # Acquire frame and resize to expected shape [1xHxWx3]
    frame = frame1.copy()

    # Acquire frame and resize to expected shape [1xHxWx3]
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_data = np.expand_dims(frame_rgb, axis=0)

    # Perform the actual detection by running the model with the image as input
    interpreter.set_tensor(input_details[0]['index'],input_data)
    interpreter.invoke()

    # Retrieve detection results
    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

    # Loop over all detections and draw detection box if confidence is above minimum threshold
    for i in range(len(scores)):

        # If all elements in the array are below the threshold, skip
        if all(score < min_conf_threshold for score in scores):
            # Default values outside of the frame to indicate that nothing was detected
            red_center = (width + 100, height + 100)
            yellow_center = (width + 100, height + 100)
            green_center = (width + 100, height + 100)
            robot_center = (width + 100, height + 100)
            continue

        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1,(boxes[i][0] * height)))
            xmin = int(max(1,(boxes[i][1] * width)))
            ymax = int(min(height,(boxes[i][2] * height)))
            xmax = int(min(width,(boxes[i][3] * width)))

            # Get detected class name
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index

            # Update flags after analyzing an element in the scores list
            # Integer division is used for the line() function later
            if object_name == 'red_cone':
                red_flag = True
                red_center = ((xmin + xmax) // 2, (ymin + ymax) // 2)
            elif object_name == 'yellow_cone':
                yellow_flag = True
                yellow_center = ((xmin + xmax) // 2, (ymin + ymax) // 2)
            elif object_name == 'green_cone':
                green_flag = True
                green_center = ((xmin + xmax) // 2, (ymin + ymax) // 2)
            elif object_name == 'robot':
                robot_flag = True   
                robot_center = ((xmin + xmax) // 2, (ymin + ymax) // 2)                   

            if debug or pics:
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), white, 2)

                # Draw label
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'red_cone: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), white, cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, black, 2) # Draw label text

    # Send UART commands based on the flags each frame
    if robot_flag and green_flag:
        data = "g".encode()
    elif robot_flag and yellow_flag:
        data = "y".encode()
    elif robot_flag and red_flag:
        data = "r".encode()
    else:
        data = "p".encode()

    try:
        BTserialChannel.write(data)  
    except:
        # Close
        os.system("echo 'ERORR: COULDNT SEND'")
        cv2.destroyAllWindows()
        videostream.stop()
        exit()

    if debug:
        # Print data
        print(f"sending: {data}")     

    if debug or pics:
        # Calculate framerate
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

        # Draw framerate in corner of frame
        cv2.putText(frame, 'FPS: {0:.2f}'.format(fps),(30,50), cv2.FONT_HERSHEY_SIMPLEX,1, cyan, 2, cv2.LINE_AA)

        # Draw lines based on the flags each frame
        if robot_flag and green_flag:
            if (0 <= robot_center[0] <= width) and (0 <= robot_center[1] <= height) and (0 <= green_center[0] <= width) and (0 <= green_center[1] <= height):
                cv2.line(frame, robot_center, green_center, green, 2)
        if robot_flag and yellow_flag:
            if (0 <= robot_center[0] <= width) and (0 <= robot_center[1] <= height) and (0 <= yellow_center[0] <= width) and (0 <= yellow_center[1] <= height):
                cv2.line(frame, robot_center, yellow_center, yellow, 2)
        if robot_flag and red_flag:
            if (0 <= robot_center[0] <= width) and (0 <= robot_center[1] <= height) and (0 <= red_center[0] <= width) and (0 <= red_center[1] <= height):
                cv2.line(frame, robot_center, red_center, red, 2)

        # If pics mode
        if pics:
            filename = os.path.join("recordings", flight_folder, f"pic_{pics_counter}.jpg")
            cv2.imwrite(filename, frame)
            pics_counter = pics_counter + 1

    if debug:
        # Display results on the frame
        cv2.imshow('Object Detector', frame)

        if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' to exit
            break
    else:
        time.sleep(0.001)
        if keyboard.is_pressed('esc'):
            break
# Close
cv2.destroyAllWindows()
videostream.stop()