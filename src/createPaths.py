import numpy as np
import os
import drone
from drone import checkBounds

SCENE_DIRECTORY = "../scenes/"
PATH_DIRECTORY = "../paths"

'''
Gives drones their inital positions from the file passed to it
'''
def initDrones(init_scene_file):   

    try:
        scenef = open(init_scene_file, "r")
    except FileNotFoundError:
        print("Make sure the Scene files exist and their location is correct.")
        print(f"Currently reading from: {init_scene_file}")
        print("                     ^^^^^^\nIf this file isn't correct, edit the SCENE_DIRECTORY variable in src/createPaths.py")
        exit()

    drones = [] 

    for line in scenef:
        line = line.strip()

        if not line: #if the line is empty 
            continue

        try:
            init_posx, init_posy, init_posz = line.split(',')
        except ValueError:
            print(f"Make sure every line in {scenef} has a point in the format: x,y,z")
            exit()
        
        init_pos = np.array([float(init_posx), float(init_posy), float(init_posz)])

        drones.append(drone.drone(init_pos))
        
    return drones

'''
Creates the paths between the drones' current position and the Scene passed to it
'''
def makePathsForScene(drones, scene_file):

    maxStepLength = 5 #Speed limit

    #Resetti spagetti (of the drones' stored path points)
    for drone in drones:
        drone.arrived = False
        drone.pathPoints = [drone.pos]

    assignTargetPosFromFile(drones, scene_file)

    ids = np.array([d.id for d in drones]) #same thing for id's

    while any(not d.arrived for d in drones): #keep going while any drones are not at their target position 

        for drone in drones:
            if not drone.arrived:
                drone.stepTowardsTarget(maxStepLength)

        next_positions = np.array([d.next_step for d in drones])

        for drone in drones:
            drone.adjustStep(next_positions, ids)

        positions = np.array([d.pos for d in drones])
        checkBounds(positions)

        drone.frame += 1
    print(f"Finished reading from {scene_file}")
    postPathCheck(drones)
    

'''
Parses the Scene file and assigns each of the drones a target position
'''
def assignTargetPosFromFile(drones, scene_file):
    
    scenef = open(scene_file, "r")
    numDrones = len(drones)  
    step = 0
    for line in scenef: 
        line = line.strip()
        if not line: #if the line is empty 
            continue
        if step >= numDrones: #Hopefully we don't need this, but prevents breaking when the number of drones in a scene is greater than the stored drones
            print("WARNING: too many drones in scene file; they will freeze in place for the current path")
            break

        try:
            target_posx, target_posy, target_posz = line.split(',')
        except ValueError:
            print(f"Make sure every line in {scene_file} has a point in the format: x,y,z")
            exit()
        
        target_pos = np.array([float(target_posx), float(target_posy), float(target_posz)])
        
        drones[step].target_pos = target_pos
        step += 1


'''
Writes all of the drones' path to a specified file
'''
def writePaths(drones, targf):
    targetf = open(targf, "w")
    
    #write paths to file
    for drone in drones:
        toWrite = ""
        for point in drone.pathPoints:

            #writes the path points with 3 decimal precision
            toWrite += f"{point[0]:.3f},{point[1]:.3f},{point[2]:.3f}:"

        targetf.write(toWrite)
        targetf.write("\n")
    targetf.close()

def postPathCheck(drones):
    all_paths = np.array([d.pathPoints for d in drones]) 
    collision = False
   
    for stepid in range(all_paths.shape[1]):
        all_positions = all_paths[0:, stepid, 0:]
        
        for i, point in enumerate(all_positions):
            if i >= all_positions.shape[0]:
                break
            for nextpoint in all_positions[(i+1):]:
                distance = np.linalg.norm(point - nextpoint)
                if distance < drones[0].radius:
                    print("---COLLISION DETECTED AFTER PATH FORMATION---\nAborting...")
                    exit()
                    collision = True

    if not collision:
       print("No collision detected")
                

#Basically main
def createAllPaths():
    sceneDir = SCENE_DIRECTORY
    pathDir = PATH_DIRECTORY

    drones = initDrones(os.path.join(sceneDir, "initScene")) 

    sceneNum = 0

    #if we were to get paths in this loop the order would be up to the operating system (i.e. bad)
    for fileName in os.listdir(sceneDir):
        sceneNum += 1

    #Runs everything for an arbitrary number of scenes
    for i in range(sceneNum - 2):
        fileName = f"Scene{i}"
        fullPath = os.path.join(sceneDir, fileName)
        makePathsForScene(drones, fullPath)
        writePaths(drones, os.path.join(pathDir, f'pathsForScene{(i)}.txt'))

    #No real reason to do this outside of the loop, but it will probably be easier to maintain an identical start and end point if they are named differrent
    fileName = "returnScene"
    fullPath = os.path.join(sceneDir, fileName)
    makePathsForScene(drones, fullPath)
    writePaths(drones, os.path.join(pathDir, f'pathsForScene{sceneNum - 2}.txt'))


            
createAllPaths()

