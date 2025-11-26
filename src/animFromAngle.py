import drone
import os
import numpy as np
import matplotlib

matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation 
from matplotlib.animation import PillowWriter


DRONE_PATH_DIRECTORY = "../paths"


'''
Reconstructs the drones' path from a pathFromScene{x}.txt file
'''
def getPathPoints(file):
    drones = []
    
    with open(file) as cordf:
        for line in cordf: 
            line = line.strip()
            if not line:
                continue
                
            pointStrs = [point.strip() for point in line.split(':') if point.strip()]
            
            points_for_this_drone = []
            current_drone = None
            
            for i, pointStr in enumerate(pointStrs):
                try:
                    parts = [float(p) for p in pointStr.split(',')]
                    if len(parts) != 3:
                        continue 
                        
                    x, y, z = parts
                        
                except ValueError:
                    print(f"Skipping bad point string: {pointStr}")
                    continue

                points_for_this_drone.append([x, y, z])
                
                if i == 0:
                    current_drone = drone.drone(np.array([x, y, z]))
            
            if points_for_this_drone and current_drone is not None:
                current_drone.pathPoints = np.array(points_for_this_drone)
                drones.append(current_drone)
                
    return drones

pathDir = DRONE_PATH_DIRECTORY
path_len = 0
twodDroneArr = []

numPaths = 0
for fileName in os.listdir(pathDir):
   numPaths += 1

#gets the path for each scene
for i in range(numPaths):
    fileName = f"pathsForScene{i}.txt"
    outf = os.path.join(pathDir, fileName)

    drones = getPathPoints(outf)
    if not drones:
        print('No drones')
        exit()

    path_len += drones[0].pathPoints.shape[0]
    twodDroneArr.append(drones)

#smashes it into a matrix and row-reduce (jk)
pointMatrixDims = (len(twodDroneArr[0]), path_len, 3)
pointMatrix = np.zeros(pointMatrixDims) 

start_col = 0

for j, drones in enumerate(twodDroneArr):
    if not drones:
        continue

    end_col = start_col + drones[0].pathPoints.shape[0]

    for i, drone in enumerate(drones):
        pointMatrix[i, start_col : end_col, :] = drone.pathPoints

    start_col = end_col
    

# --- Animation Setup ---

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.view_init(elev=45, azim=45, roll=0)

if pointMatrix.size > 0:
    # Find min/max across ALL drones and ALL frames (axis 0 and 1)
    min_coords = pointMatrix.min(axis=(0, 1)) 
    max_coords = pointMatrix.max(axis=(0, 1))
    
    center = (max_coords + min_coords) / 2.0
    
    # Find the largest range needed for any axis
    max_range = (max_coords - min_coords).max()
    
    if max_range < 1e-5:
        max_range = 10.0 # Default zoom
        
    plot_radius = max_range * 0.55 # 10% padding
    
    print(min_coords)
    print(max_coords)
    ax.set_xlim(min_coords[0], max_coords[0])
    ax.set_ylim(min_coords[1], max_coords[1])
    ax.set_zlim(min_coords[2], max_coords[2])

scatters = [ax.scatter([], [], [], marker='o', s=5) for _ in range(len(drones))]


def update(frame):
    global scatters
    
    frame = frame * 1
    if frame > path_len:
        print('End animation')
        exit()

    # 1. Update drone positions
    for row, scatter in enumerate(scatters):
        x, y, z = pointMatrix[row, frame]
        scatter._offsets3d = ([x], [y], [z])

    ax.set_title(f'Drone Path Animation - Frame {frame+1}/{path_len}')
    
    return scatters
      

ani = FuncAnimation(fig=fig, func=update, frames=path_len, interval=1)
plt.show()
