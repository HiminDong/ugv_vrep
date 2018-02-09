import vrep
import sys
from time import sleep
from pynput.keyboard import Key, Listener
import numpy as np
import cv2
import pickle

heading = 0
velocity = 0

def bound(val):
    upperBound = 1
    lowerBound = -1
    if val >= upperBound:
        val = upperBound
    elif val <= lowerBound:
        val = lowerBound
    return val

def on_press(key):
    global velocity, heading, handles
    if key == Key.up: # Forward motion
        velocity += 0.1
    elif key == Key.down: # Backward motion
        velocity -= 0.1
    elif key == Key.left: # Backward motion
        heading += 0.08
    elif key == Key.right: # Backward motion
        heading -= 0.08
    elif key == Key.space:
        velocity = 0
        heading = 0
    heading = bound(heading)
    velocity = bound(velocity)
    print("Heading= ", heading, " - Velocity= ", velocity)
    set_motion(heading, velocity, handles)

def on_release(key):
    pass

def set_motion(left, right, handles):
    global clientID

    left = -10*(velocity - heading)
    right =  -10*(velocity + heading)
    
    vrep.simxSetJointTargetVelocity(clientID, handles[0], left, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, handles[1], left, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, handles[2], right, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, handles[3], right, vrep.simx_opmode_streaming)

###

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print("Connection cannot be established. Exiting")
    sys.exit('Could not connect')

## Initiate handles
# Motor
errCode, FL_handle = vrep.simxGetObjectHandle(clientID, "FL", vrep.simx_opmode_blocking)
errCode, RL_handle = vrep.simxGetObjectHandle(clientID, "RL", vrep.simx_opmode_blocking)
errCode, FR_handle = vrep.simxGetObjectHandle(clientID, "FR", vrep.simx_opmode_blocking)
errCode, RR_handle = vrep.simxGetObjectHandle(clientID, "RR", vrep.simx_opmode_blocking)
handles = [FL_handle, RL_handle, FR_handle, RR_handle]

# Front cam
errCode, cam = vrep.simxGetObjectHandle(clientID, "front_cam", vrep.simx_opmode_blocking)
print(errCode)

err, resolution, image = vrep.simxGetVisionSensorImage(clientID, cam, 0, vrep.simx_opmode_streaming)
print(err)

with Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

#listener = Listener(on_press=on_press, on_release=on_release)
#listener.start()

data_combined = []
i = 0
four_floats = []
while True:
    # err, resolution, image = vrep.simxGetVisionSensorImage(clientID, cam, 0, vrep.simx_opmode_streaming)
    # print(err, resolution)
    # sleep(0.5)
    # if image:
    #     image = np.array(image,dtype=np.uint8)
    #     image.resize([resolution[1],resolution[0],3])
    #     cv2.imwrite('savedim.png', cv2.flip(image, 0))

    # Lidar data
    # err, ints, floats, strings, outBuffer = vrep.simxCallScriptFunction(clientID, "velodyneVPL_16", 1, "getVelodyneData_function", [], [], [], bytearray(), vrep.simx_opmode_blocking)
    
    #print(len(floats))
    #if len(floats) == 48000:
    #    four_floats.append(floats)
    #    i += 1

    #if i == 4: #change it to 4 for 360

    #    print("writing")
    #    with open('pointcloud.txt', 'w') as write_file:
    #        for group in four_floats:
    #            for x in group:
    #                write_file.write(str(x) + "\n")
    #    print("done writing")
    #    break
    pass



listener.stop()
print("Stopped listening. Closing")
