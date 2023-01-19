''' Python implementation of the Predictive Avoidance Model (PAM)
    from 
    A Predictive Collision Avoidance Model for Pedestrian Simulation,
    I. Karamouzas, P. Heil, P. van Beek, M. H. Overmars
    Motion in Games (MIG 2009), Lecture Notes in Computer Science (LNCS), Vol. 5884, 2009
'''

from dataclasses import dataclass
import numpy as np
from scipy.spatial import distance

@dataclass
class Parameters:

    # The agents field of view
    field_of_view = 200.0
    # The agents radius ? Used here in this implementation or in sim?
    agent_radius = 0.5
    # Minimum agent distance
    min_agent_dist = 0.1
    # the mid distance parameters in peicewise personal space function predictive force dist
    dmid = 4.0
    # KSI  
    ksi = 0.5
    # Nearest Neighbour distance ? Used here in this implementation or in sim?
    neighbor_dist = 10.0
    # Maximum neighbours to consider ? Used here in this implementation or in sim?
    max_neighbors = 3
    # Maximum acceleration ? Used here in this implementation or in sim/physics?
    max_accel = 20.0
    # Maximum speed
    max_speed = 7
    # Preferred Speed
    preferred_vel = 2.5
    # Goal acquired radius 
    goal_radius = 1.0
    # Time Horizon
    time_horizon = 4.0
    # Agent Distance
    agent_dist = 0.1
    # Wall Distance
    wall_dist = 0.1
    # Wall Steepnes
    wall_steepness = 2.0
    # Agent Strength
    agent_strength = 1.0
    # wFactor, factor to progressively scale down forces in when in a non-collision state
    w_factor = 0.8
    # Noise flag (should noise be added to the movement action)
    noise = False 
    force_clamp = 40.0
    
    # *private* Ideal wall distance
    _ideal_wall_dist = agent_radius + wall_dist
    # *private* Squared ideal wall distance
    _SAFE = _ideal_wall_dist * _ideal_wall_dist
    # *private* Agent Personal space
    _agent_personal_space = agent_radius + min_agent_dist
    # *private* the min distance parameters in peicewise personal space function
    _dmin = agent_radius + _agent_personal_space
    # *private* the max distance parameters in peicewise personal space function
    _dmax = time_horizon * max_speed
    # *private* FOV cosine
    _cosFOV = np.cos((0.5 * np.pi * field_of_view) / 180.0)
    
def ray_intersects_disc(pi, pj, v, r):
    # calc ray disc est. time to collision
    t = 0.0
    w = pj - pi
    a = np.dot(v, v)
    b = np.dot(w, v)
    c = np.dot(w, w) - (r * r)
    discr = (b * b) - (a * c)
    if discr > 0.0:
        t = (b - np.sqrt(discr)) / a
        if t < 0.0:
            t = 999999.0
    else:
        t = 999999.0
    
    return t

def mag(v):
    # calc magnitude of vector
    v_mag = np.sqrt(v.dot(v))
    return v_mag 

def norm(v):
    # normalize a vector
    v_norm = np.array([0, 0, 0], dtype='float64')
    magnitude = mag(v)
    if magnitude > 0.0:
        v_norm = v / magnitude
    return v_norm

def get_neighbors(cur, agents, pn_r):
    dist = distance.cdist([cur], agents)
    pn = dist < pn_r
    # Index to remove is when its zero
    pn_self = dist == 0
    pn_self = np.nonzero(pn_self)
    pn[pn_self] = False

    pn = np.nonzero(pn)
    return pn


def wall_force(obstacles, rr_i, closest_point, SAFE, add_force):

    for i in range(len(obstacles)):
        # Step 1: get closest point on obstacle to agent
        # [[ Need python code for this in simulation ]]
        
        n_w = rr_i - closest_point
        d_w = mag(n_w) * mag(n_w)

        if (d_w < SAFE):
            d_w = np.sqrt(d_w)
            if (d_w > 0):
                n_w /= d_w
            if ((d_w - Parameters.agent_radius) < 0.001):
                dist_min_radius =  0.001
            else: 
                d_w - Parameters.agent_radius
            obstacle_force = (Parameters._ideal_wall_dist - d_w) / np.pow(dist_min_radius, Parameters.wall_steepness) * n_w
            add_force(obstacle_force)
    

def calc_goal_force(goal, rr_i, vv_i):
    # Preferred velocity is preferred speed in direction of goal
    preferred_vel = Parameters.preferred_vel * norm(goal - rr_i)
    
    # Goal force, is always added
    goal_force = (preferred_vel - vv_i) / Parameters.ksi

    return goal_force

def collision_param(rr_i, vv_i, desired_vel, pn_rr, pn_vv, pn_r):
    # Keep track of if we ever enter a collision state
    agent_collision = False

    t_pairs = []
    # Handle agents tc values for predictive forces among neighbours
    for j, rr_j in enumerate(pn_rr):
        #  Get position and velocity of neighbor agent
        vv_j = pn_vv[j]

        #  Get radii of neighbor agent
        rj = pn_r[j]
        
        combined_radius = Parameters._agent_personal_space + rj
        
        w = rr_j - rr_i 
        if (mag(w) < combined_radius):
            agent_collision = True
            t_pairs.append((0.0, j))
        else:
            rel_dir = norm(w)
            if np.dot(rel_dir, norm(vv_i)) < Parameters._cosFOV:
                continue
                
            tc = ray_intersects_disc(rr_i, rr_j, desired_vel - vv_j, combined_radius)
            if tc < Parameters.time_horizon:
                if len(t_pairs) < Parameters.max_neighbors:
                    t_pairs.append((tc, j))
                elif tc < t_pairs[0][0]:
                    t_pairs.pop()
                    t_pairs.append((tc, j))

    return t_pairs, agent_collision

def predictive_force(rr_i, desired_vel, desired_speed, pn_rr, pn_vv, pn_r, vv_i):
    # Handle predictive forces// Predictive forces

    # Setup collision parameters
    t_pairs, agent_collision = collision_param(rr_i, vv_i, desired_vel, pn_rr, pn_vv, pn_r)

    # This will be all the other forces, added in a particular way
    steering_force = np.array([0, 0, 0], dtype='float64')

    # will store a list of tuples, each tuple is (tc, agent)
    force_count = 0

    for t_pair in t_pairs:
        # Nice variables from the t_pair tuples
        t = t_pair[0]
        agent_idx = t_pair[1]
        
        force_dir = rr_i + (desired_vel * t) - pn_rr[agent_idx] - (pn_vv[agent_idx] * t)
        force_dist = mag(force_dir)
        if force_dist > 0:
            force_dir /= force_dist
            
        collision_dist = np.maximum(force_dist - Parameters.agent_radius - pn_r[agent_idx], 0.0)
        
        #D = input to evasive force magnitude piecewise function
        D = np.maximum( (desired_speed * t) + collision_dist, 0.001)
        
        force_mag = 0.0
        if D < Parameters._dmin:
            force_mag = Parameters.agent_strength * Parameters._dmin / D
        elif D < Parameters.dmid:
            force_mag = Parameters.agent_strength
        elif D < Parameters._dmax:
            force_mag = Parameters.agent_strength * (Parameters._dmax - D) / (Parameters._dmax - Parameters.dmid)
        else:
            continue
            
        force_mag *= np.power( (1.0 if agent_collision else Parameters.w_factor), force_count)
        force_count += 1
        steering_force = force_mag * force_dir

    return steering_force

def add_noise(steering_force):
    angle = np.random.uniform(0.0, 1.0) * 2.0 * np.pi
    dist = np.random.uniform(0.0, 1.0) * 0.001
    steering_force += dist * np.array([np.cos(angle),np.sin(angle),0], dtype='float64')

    return steering_force

def compute_force(rr_i, ri, vv_i, mass, goal, pn_rr, pn_vv, pn_r, dt):

    # Get the goal force
    goal_force = calc_goal_force(goal, rr_i, vv_i)

    # Desired values if all was going well in an empty world
    desired_vel = vv_i + goal_force * dt
    desired_speed = mag(desired_vel)

    # Get obstacle (wall) forces
    obstacle_force = np.array([0, 0, 0], dtype='float64')
    #@TODO 
    # obstacle_force = wall_force()
    
    # Get predictive steering forces
    steering_force = predictive_force(rr_i, desired_vel, desired_speed, pn_rr, pn_vv, pn_r, vv_i)

    # Add noise for reducing deadlocks adding naturalness
    if Parameters.noise:
        steering_force = add_noise(steering_force)

    # Clamp driving force
    if mag(steering_force) > Parameters.force_clamp:
        steering_force = norm(steering_force) *  Parameters.force_clamp
    
    return goal_force + obstacle_force + steering_force