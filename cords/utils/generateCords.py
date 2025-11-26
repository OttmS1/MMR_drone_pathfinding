import math
import os
import random as rand
global numDrones
numDrones = 50 

scene_directory = "scenes"

def writeInitDrones():
    with (open(os.path.join("../../scenes", "initScene"), "w+")) as file:
        delimiter = ","
        droneSpacing = 20
        for i in range(numDrones):
            toWrite = str(((i % 10) + 10) * droneSpacing) + delimiter + str(math.floor(i/10) * droneSpacing + 10) + delimiter + str(10) + '\n'

            file.write(toWrite)
    with (open(os.path.join("../../scenes", "returnScene"), "w+")) as file:
        delimiter = ","
        droneSpacing = 20
        for i in range(numDrones):
            toWrite = str(((i % 10) + 10) * droneSpacing) + delimiter + str(math.floor(i/10) * droneSpacing + 10) + delimiter + str(10) + '\n'

            file.write(toWrite)


def writeRandomCords(num):
    with (open(os.path.join("../../scenes", f"randomCords{num}"), "w")) as file:
        delimiter = ","
        for i in range(numDrones):
            toWrite = str(rand.randint(1, 1000)) + delimiter + str(rand.randint(1, 1000)) + delimiter + str(rand.randint(1, 1000)) + '\n'

            file.write(toWrite) 


writeInitDrones()
