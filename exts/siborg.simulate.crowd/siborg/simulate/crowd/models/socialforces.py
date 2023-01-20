from dataclasses import dataclass
import numpy as np
from scipy.spatial import distance

# zero_vec = np.array([0,0,0], dtype='float64')

@dataclass
class Parameters:
    # names from https://www.sciencedirect.com/science/article/pii/S0378437120306853

    Tau = 0.5 #(s)
    A = 2000.0
    B = 0.08
    kn = 1.2 * 100_000 # Kgs^-2
    kt = 2.4 * 100_000 # Kg m^-1 s^-1
    max_speed = 10
    v_desired = 3.5

def calc_wall_force():
    # TODO add wall and geometry recognition 

    force = np.array([0,0,0], dtype='float64')
    return force 

def calc_agent_force(rr_i, ri, vv_i, pn_rr, pn_vv, pn_r):
    #  Sum the forces of neighboring agents 
    force = np.array([0,0,0], dtype='float64')

    #  Set the total force of the other agents to zero
    ff_ij = np.array([0,0,0], dtype='float64')

    rr_j =np.array([0,0,0], dtype='float64')

    #  Iterate through the neighbors and sum (f_ij)
    for j, rr_j in enumerate(pn_rr):

        #  Get position and velocity of neighbor agent
        vv_j = pn_vv[j]

        #  Get radii of neighbor agent
        rj = pn_r[j]

        #  Pass agent position to AgentForce calculation
        ff_ij = neighbor_force(rr_i, ri, vv_i, rr_j, rj, vv_j)

        #  Sum Forces
        force += ff_ij
    
    return force
    
def neighbor_force(rr_i, ri, vv_i, rr_j, rj, vv_j):
    #  Calculate the force exerted by another agent
    #  Take in this agent (i) and a neighbors (j) position and radius

    #  Sum of radii
    rij = ri + rj
    #  distance between center of mass
    d_ij = mag(rr_i - rr_j)

    #  "n_ij is the normalized vector points from pedestrian j to i"
    n_ij = norm(rr_i - rr_j) # Normalized vector pointing from j to i

    #  t_ij "Vector of tangential relative velocity pointing from i to j." 
    #  A sliding force is applied on agent i in this direction to reduce the relative velocity.
    t_ij = np.cross(vv_j - vv_i, [0,0,1] )
    dv_ji = np.dot(vv_j - vv_i, t_ij)

    #  Calculate f_ij
    force =  repulsion(rij, d_ij, n_ij) + proximity(rij, d_ij, n_ij) + sliding(rij, d_ij, dv_ji, t_ij)

    return force

def calc_goal_force(goal, pos, vel, mass, v_desired, dt):
    ee_i = norm(goal - pos)
    force = mass * ( ( (v_desired * ee_i) - vel ) / Parameters.Tau )
    return force 


def G(r_ij, d_ij):
    # g(x) is a function that returns zero if pedestrians touch
    # otherwise is equal to the argument x 
    if (d_ij > r_ij): return 0.0
    return r_ij - d_ij;

def repulsion(r_ij, d_ij, n_ij):
    force = Parameters.A * np.exp( (r_ij - d_ij) / Parameters.B) * n_ij
    return force

def proximity(r_ij, d_ij, n_ij):
    force = Parameters.kn * G(r_ij, d_ij) * n_ij
    return force 

def sliding(r_ij, d_ij, dv_ji, t_ij):
    force = Parameters.kt * G(r_ij, d_ij) * (dv_ji * t_ij)
    return force

def mag(v):
    # calc magnitude of vector
    v_mag = np.sqrt(v.dot(v))
    return v_mag 

def norm(v):
    # normalize a vector
    v_norm = v / mag(v)
    return v_norm

def get_neighbors(cur, agents, pn_r):
    dist = distance.cdist([cur], agents)
    pn =  dist < pn_r
    # Index to remove is when its zero
    pn_self = dist == 0
    pn_self = np.nonzero(pn_self)
    pn[pn_self] = False

    pn = np.nonzero(pn)
    return pn

def compute_force(rr_i, ri, vv_i, mass, goal, pn_rr, pn_vv, pn_r, dt):
    # Get the force for this agent to the goal
    goal = calc_goal_force(goal, rr_i, vv_i, mass, Parameters.v_desired, dt)

    agent = calc_agent_force(rr_i, ri, vv_i, pn_rr, pn_vv, pn_r)

    wall = calc_wall_force()

    force = goal + agent + wall

    force = norm(force) *  min(mag(force), Parameters.max_speed)

    return force


