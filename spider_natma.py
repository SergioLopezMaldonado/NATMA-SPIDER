import numpy as np
import math
body_vector = [0,1,0]
legs_data = {"leg1":{"z":[15],"y":[14,13],"current_angle":90},
        "leg2":{"z":[15],"y":[14,13],"current_angle":90},
        "leg3":{"z":[15],"y":[14,13],"current_angle":90},
        "leg4":{"z":[15],"y":[14,13],"current_angle":90},    
        "leg5":{"z":[15],"y":[14,13],"current_angle":90},    
        "leg6":{"z":[15],"y":[14,13],"current_angle":90}}
def body_rotation(direction_vector):
    rotation = np.cross(body_vector, direction_vector)[2]
    if rotation == 0:
        if  np.array_equal(np.multiply(-1, body_vector),direction_vector):
            return -1
    return rotation
def angle_between_vector(vec1,vec2):
    unit1 = vec1 / np.linalg.norm(vec1)
    unit2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(unit1, unit2)
    angle = int(math.degrees(math.acos(dot_product)))
    if angle > 90:
        angle = 90-(angle -180)
    return angle
def servo_rotation_factor(leg, axis, rotation):
    
    K = 0
    if axis == "y":
        K = 1
        if leg[0] > len(legs_data)/ 2:
            K = -1
    else:
        if rotation == -1:
            K = 1
        elif rotation == 1:
            K = -1
        else:
            K = -1
            if leg[0] > len(legs_data)/ 2:
                K = 1
    return K


def servomove(pin, angle):
    return 
def delay():
    return 





def move_legs(joints,angle,axis, rotation):

    pins = list()
    K2 = 1
    angle = int(angle)

    if angle < 0:
        angle = -1*angle
        K2 = -1

    for joint in joints:
        pin = legs_data[joint[1]]["y"][0]
        K = servo_rotation_factor(joint, axis, rotation)
        pins.append([pin, K])

    for theta in range(0, angle):
        for pin in pins:
            K = K2*pin[1]
            servomove(pin[0], 90 + K*theta)
        delay()

    return

def move_body(direction_vector, step_size):

    first_set_legs = list()
    second_set_legs = list()
    for leg in legs_data:
        num_leg = int(leg[-1])
        if num_leg  % 2 == 0:
            first_set_legs.append([num_leg ,leg])
        else:
            second_set_legs.append([num_leg ,leg])   
    rotation = body_rotation(direction_vector)
    print(rotation)
    angle = angle_between_vector(body_vector, direction_vector)
    print(angle)
    degrees = 0
    while True:
        print(degrees)
        if input("parar\n") == "si":
            break;
        if degrees > angle:
            rotation = 0
            degrees = 0
        move_legs(first_set_legs,step_size,"y", rotation)
        if (degrees != 0):
            move_legs(second_set_legs,-1*step_size,"z", rotation)
        move_legs(first_set_legs,step_size,"z", rotation)
        move_legs(first_set_legs,-1*step_size,"y", rotation)

        move_legs(second_set_legs,step_size,"y", rotation)
        move_legs(first_set_legs,-1*step_size,"z", rotation)
        move_legs(second_set_legs,step_size,"z", rotation)
        move_legs(second_set_legs,-1*step_size,"y", rotation)
        degrees += step_size
    return

move_body([1,1,0],10)

