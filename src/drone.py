import numpy as np
import math

upper_bounds_array = np.array([1000, 1000, 1000])
lower_bounds_array = np.array([0, 0, 0])

def checkBounds(all_positions):

    upper_bound_mask = (all_positions > upper_bounds_array)
    lower_bound_mask = (all_positions < lower_bounds_array) 
    if np.any(upper_bound_mask) or np.any(lower_bound_mask):
        print(f"Drone out of bounds. Terminating execution.")
        for i, outside in enumerate(upper_bound_mask):
            if outside.any():
                print(f"Offender: id: {i} with pos {all_positions[i]}")
        for i, outside in enumerate(lower_bound_mask):
            if outside.any():
                print(f"Offender: id: {i} with pos {all_positions[i]}")
        exit()

class drone: 
    numDrones = 0
    frame = 0 
    def __init__(self, initPos):
        self.pos = np.array(initPos, dtype=float)
        self.prevPos = self.pos 
        self.target_pos = np.array(initPos, dtype=float) 
        self.pathPoints = [self.pos]  
        self.next_step = np.array([0, 0, 0])
        self.radius = 1.0
        self.arrived = False
        self.id = drone.numDrones
        drone.numDrones += 1

    def stepTowardsTarget(self, distance_per_step=1.0):
        self.prevPos = self.pos
        
        toTarget = self.target_pos - self.pos
        distance_to_target = np.linalg.norm(toTarget)
        
        if distance_to_target < distance_per_step:
            self.arrived = True
            self.pos = self.target_pos # Snap to target
            self.next_step = self.pos
            self.pathPoints.append(self.pos)
            return

        unit_to_target = toTarget / distance_to_target #unit vector in direction of the target 
        self.next_step = self.pos + unit_to_target * distance_per_step


    def adjustStep(self, all_next_positions, all_ids):

        collision = self.checkCollision(all_next_positions, all_ids)
        is_oob = np.any(self.next_step < lower_bounds_array) or np.any(self.next_step > upper_bounds_array)

        maxIters = 10 #how many times we try to prevent a collision before giving up 
        iters = 0
        repulseMod = 1.0 

        while (collision or is_oob) and iters < maxIters:
            self.next_step = self.applyRepulse(all_next_positions, all_ids, repulseMod)
            
            collision = self.checkCollision(all_next_positions, all_ids)
            
            iters += 1

        if collision or is_oob: # Check if we failed to fix the issue
            print(f"Unsafe path for drone {self.id} on frame {drone.frame}. Staying put.")
            self.pos = self.prevPos
            self.next_step = self.prevPos
        else:
            self.pos = np.maximum(self.next_step, lower_bounds_array)
            self.pos = np.minimum(self.pos, upper_bounds_array)
            self.next_step = self.pos 

        self.pathPoints.append(self.pos)

    def applyRepulse(self, all_positions, all_ids, repulseMod):
        
        other_to_self_vec = self.next_step - all_positions  # (N, 3) array
        
        distanceMag = np.linalg.norm(other_to_self_vec, axis=1) # (N,) array

        not_self_mask = (all_ids != self.id)

        in_range_mask = (distanceMag <= self.radius * 15) & (distanceMag > 0) 

        valid_mask = not_self_mask & in_range_mask
        
        if not np.any(valid_mask):
            return self.next_step 

        valid_vectors = other_to_self_vec[valid_mask]
        valid_distances = distanceMag[valid_mask]

        # (N,) -> (N, 1)
        repulseMagnitude = repulseMod / np.array([(math.exp((x / 2) - 5) + 1) for x in valid_distances])
        
        # (N, 3) / (N, 1) = (N, 3) (unit vectors)
        other_to_self_vecUnit = valid_vectors / valid_distances[:, np.newaxis] 
        
        # (N, 3) * (N, 1) = (N, 3) (force vectors)
        repulsion_vectors = other_to_self_vecUnit * repulseMagnitude[:, np.newaxis]

        # Sum all (N, 3) vectors into one (1, 3) vector
        drone_repulsion_vec = np.sum(repulsion_vectors, axis=0)

        boundary_repulsion_vec = np.zeros(3, dtype=float)
        activation_range = 50.0 # Start repelling 50 units from 0

        for dim in range(3):
            if self.next_step[dim] < activation_range:
                penetration = activation_range - self.next_step[dim]
                magnitude = repulseMod * penetration / activation_range
                boundary_repulsion_vec[dim] += magnitude

            if self.next_step[dim] > upper_bounds_array[dim] - activation_range:
                dist_to_upper = upper_bounds_array[dim] - self.next_step[dim]
                penetration = activation_range - dist_to_upper
                magnitude = repulseMod * penetration / activation_range
                boundary_repulsion_vec[dim] -= magnitude 
        total_repulsion_vec = drone_repulsion_vec + boundary_repulsion_vec
        
        total_mag = np.linalg.norm(total_repulsion_vec)
        if total_mag > repulseMod * 2: 
            total_repulsion_vec = (total_repulsion_vec / total_mag) * (repulseMod * 2)

        return self.next_step + total_repulsion_vec


    def checkCollision(self, all_positions, all_ids):
        
        other_to_self_vec = self.next_step - all_positions
        distance_mag = np.linalg.norm(other_to_self_vec, axis=1)
        
        not_self_mask = (all_ids != self.id) #Array of booleans representing whether or not they are the drone
        collision_mask = (distance_mag < self.radius * 2) #Array of booleans representing whether or not they are in the collision bubble
        
        if np.any(collision_mask & not_self_mask):
            return True
            
        return False
