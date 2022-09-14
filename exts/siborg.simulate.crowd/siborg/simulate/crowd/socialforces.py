from dataclasses import dataclass
import numpy as np
from scipy.spatial import distance

zero_vec = np.array([0,0,0], dtype='float64')

@dataclass
class Parameters:

    T = 0.5
    A = 2000.0
    B = 0.08
    k = 1.2 * 100000
    Kappa = 2.4 * 100000

    max_speed = 20

    v_desired = 2.5

def calc_wall_force():
    # TODO add wall and geometry recognition 

    force = zero_vec
    return force 

def calc_agent_force(rr_i, ri, vv_i, pn_rr, pn_vv, pn_r):
    ''' 
    rr_i : position
    ri : radius
    vv_i : velocity
    pn_rr : List[perceived neighbor positions]
    pn_vv : List[perceived neighbor velocities]
    pn_r : List[perceived neighbor radius]
    '''

    #  Sum the forces of neighboring agents 
    force = zero_vec

    #  Set the total force of the other agents to zero
    ff_ij = zero_vec

    rr_j =zero_vec
    rj = 0.0

    #  Iterate through the neighbors and sum (f_ij)
    for j, rr_j in enumerate(pn_rr):

        #  Get position and velocity of neighbor agent
        vv_j = pn_vv[j]

        #  Get radii of neighbor agent
        rj = pn_r[j]

        #  Pass agent position to AgentForce calculation
        ff_ij = agent_force(rr_i, ri, vv_i, rr_j, rj, vv_j)

        #  Sum Forces
        force += ff_ij
    
    return force


def calc_goal_force(goal, pos, vel, mass, v_desired, dt):
    return goal_force(goal, pos, vel, mass, v_desired, dt)

def agent_force(rr_i, ri, vv_i, rr_j, rj, vv_j):
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
    t_ij = vv_j - vv_i
    deltaV = np.dot(vv_j - vv_i, t_ij)

    #  Calculate f_ij
    force = ( ( proximity(rij, d_ij) + repulsion(rij, d_ij) ) * n_ij ) +  sliding(rij, d_ij, deltaV, t_ij)

    return force

def goal_force(goal, i_xyz, v_i, m_i, v_desired, dt):
    '''_summary_
    
    Vector3 goal, Vector3 i_xyz, Vector3 v_i, float m_i, float v_desired

    Parameters
    ----------
    goal : _type_
        _description_
    i_xyz : _type_
        _description_
    v_i : _type_
        _description_
    m_i : _type_
        _description_
    v_desired : _type_
        _description_

    Returns
    -------
    _type_
        _description_
    '''

    ee_i = norm(goal - i_xyz)
    force = m_i * ( ( (v_desired * ee_i) - v_i ) / (dt) ) #  alt is to replace `dt` with  Parameters.T 

    return force 


def G(r_ij, d_ij):
    # g(x) is a function that returns zero if pedestrians touch
    # otherwise is equal to the argument x 
    if (d_ij > r_ij): return 0.0
    return r_ij - d_ij;

def proximity(r_ij, d_ij):
    force = Parameters.A * np.exp( (r_ij - d_ij) / Parameters.B)
    return force

def repulsion(r_ij, d_ij):
    force = Parameters.k * G(r_ij, d_ij)
    return force 

def sliding(r_ij, d_ij, deltaVelocity, t_ij):
    force = Parameters.Kappa * G(r_ij, d_ij) * (deltaVelocity * t_ij)
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
    ''' 
    cur : agent position
    agents : List[agent positions]
    pn_r : agent perception radius 
    
    return : indices of neighbors
    '''
    dist = distance.cdist([cur], agents)
    pn =  dist < pn_r
    # Index to remove is when its zero
    pn_self = dist == 0
    pn_self = np.nonzero(pn_self)
    pn[pn_self] = False

    pn = np.nonzero(pn)
    return pn

def compute_force(rr_i, ri, vv_i, mass, goal, pn_rr, pn_vv, pn_r, dt):
    ''' 
    # agent is a position
    rr_i : position
    ri : radius
    vv_i : velocity
    pn_rr : List[perceived neighbor positions]
    pn_vv : List[perceived neighbor velocities]
    pn_r : List[perceived neighbor radius]
    '''

    # Get the force for this agent to the goal
    goal = calc_goal_force(goal, rr_i, vv_i, mass, Parameters.v_desired, dt)

    agent = calc_agent_force(rr_i, ri, vv_i, pn_rr, pn_vv, pn_r)

    wall = calc_wall_force()

    force = goal + agent + wall

    force = norm(force) *  min(mag(force), Parameters.max_speed)

    return force


