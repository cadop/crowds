from dataclasses import dataclass
import numpy as np
from scipy.spatial import distance


''' Python implementation of the Predictive Avoidance Model (PAM)
    from 
    A Predictive Collision Avoidance Model for Pedestrian Simulation,
    I. Karamouzas, P. Heil, P. van Beek, M. H. Overmars
    Motion in Games (MIG 2009), Lecture Notes in Computer Science (LNCS), Vol. 5884, 2009
'''

@dataclass
class Parameters:

    # The agents field of view
    fieldOfView = 200.0
    
    # The agents radius ? Used here in this implementation or in sim?
    agentRadius = 0.5
    
    # Minimum agent distance
    minAgentDistance = 0.1
    
    # the mid distance parameters in peicewise personal space function predictive force dist
    dmid = 4.0
    
    # KSI  
    ksi = 0.5
    
    # Nearest Neighbour distance ? Used here in this implementation or in sim?
    neighborDist = 10.0
    
    # Maximum neighbours to consider ? Used here in this implementation or in sim?
    maxNeighbors = 3
    
    # Maximum acceleration ? Used here in this implementation or in sim/physics?
    maxAccel = 20.0
    
    # Maximum speed
    maxSpeed = 2.0
    
    # Preferred Speed
    preferredSpeed = 1.3
    
    # Goal acquired radius 
    goalRadius = 1.0
    
    # Time Horizon
    timeHorizon = 4.0
    
    # Agent Distance
    agentDistance = 0.1
    
    # Wall Distance
    wallDistance = 0.1
    
    # Wall Steepnes
    wallSteepness = 2.0
    
    # Agent Strength
    agentStrength = 1.0
    
    # wFactor, factor to progressively scale down forces in when in a non-collision state
    wFactor = 0.8
    
    # Noise flag (should noise be added to the movement action)
    noise = False
    
    # *private* Ideal wall distance
    idealWallDistance = agentRadius + wallDistance
    
    # *private* Squared ideal wall distance
    SAFE = idealWallDistance * idealWallDistance
    
    # *private* Agent Personal space
    agentPersonalSpace = agentRadius + minAgentDistance

    # *private* the min distance parameters in peicewise personal space function
    dmin = agentRadius + agentPersonalSpace
    
    # *private* the max distance parameters in peicewise personal space function
    dmax = timeHorizon * maxSpeed

    # *private* FOV cosine
    cosFOV = np.cos((0.5 * np.pi * fieldOfView) / 180.0)
    
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


def compute_force(rr_i, ri, vv_i, mass, goal, pn_rr, pn_vv, pn_r, dt):
    # will store a list of tuples, each tuple is (tc, agent)
    t_pairs = []
    forceCount = 0
    
    # Preferred velocity is preferred speed in direction of goal
    preferredVelocity = Parameters.preferredSpeed * norm(goal - rr_i)
    
    # Goal force, is always added
    goalForce = (preferredVelocity - vv_i) / Parameters.ksi
    
    # Desuired values if all was going well in an empty world
    desiredVelocity = vv_i + goalForce * dt
    desiredSpeed = mag(desiredVelocity)

    # Handle obstacles
    obstacleForce = np.array([0, 0, 0], dtype='float64')
    '''
    for (int i = 0; i < obstacles.Length; ++i)
    {
        #Step 1: get closest point on obstacle to agent
        #[[ Need python code for this in simulation ]]
        
        Vector3 n_w = transform.position - closestPoint;

        float d_w = n_w.sqrMagnitude;

        if (d_w < SAFE)
        {
            d_w = Mathf.Sqrt(d_w);
            if (d_w > 0)
                n_w /= d_w;

            float distanceMinimumRadius = ((d_w - agentRadius) < 0.001f) ? 0.001f : d_w - agentRadius;
            Vector3 obstacleForce = (idealWallDistance - d_w) / Mathf.Pow(distanceMinimumRadius, wallSteepness) * n_w;
            rb.AddForce(obstacleForce, ForceMode.Force);
        }
    }
    '''
    
    # Keep track of if we ever enter a collision state
    agentCollision = False
    
    # Handle agents tc values for predictive forces among neighbours
    for j, rr_j in enumerate(pn_rr):
        #  Get position and velocity of neighbor agent
        vv_j = pn_vv[j]

        #  Get radii of neighbor agent
        rj = pn_r[j]
        
        combinedRadius = Parameters.agentPersonalSpace + rj
        
        w = rr_j - rr_i 
        if (mag(w) < combinedRadius):
            agentCollision = True
            t_pairs.append((0.0, j))
        else:
            relDir = norm(w)
            if np.dot(relDir, norm(vv_i)) < Parameters.cosFOV:
                continue
                
            tc = ray_intersects_disc(rr_i, rr_j, desiredVelocity - vv_j, combinedRadius)
            if tc < Parameters.timeHorizon:
                if len(t_pairs) < Parameters.maxNeighbors:
                    t_pairs.append((tc, j))
                elif tc < t_pairs[0][0]:
                    t_pairs.pop()
                    t_pairs.append((tc, j))
                    
    # This will be all the other forces, added in a particular way
    drivingForce = np.array([0, 0, 0], dtype='float64')
    
    # Handle predictive forces// Predictive forces
    for t_pair in t_pairs:
        # Nice variables from the t_pair tuples
        t = t_pair[0]
        agentIndex = t_pair[1]
        
        forceDirection = rr_i + (desiredVelocity * t) - pn_rr[agentIndex] - (pn_vv[agentIndex] * t)
        forceDistance = mag(forceDirection)
        if forceDistance > 0:
            forceDirection /= forceDistance
            
        collisionDistance = np.maximum(forceDistance - Parameters.agentRadius - pn_r[agentIndex], 0.0)
        
        #D = input to evasive force magnitude piecewise function
        D = np.maximum( (desiredSpeed * t) + collisionDistance, 0.001)
        
        forceMagnitude = 0.0
        if D < Parameters.dmin:
            forceMagnitude = Parameters.agentStrength * Parameters.dmin / D
        elif D < Parameters.dmid:
            forceMagnitude = Parameters.agentStrength
        elif D < Parameters.dmax:
            forceMagnitude = Parameters.agentStrength * (Parameters.dmax - D) / (Parameters.dmax - Parameters.dmid)
        else:
            continue
            
        forceMagnitude *= np.power( (1.0 if agentCollision else Parameters.wFactor), forceCount)
        forceCount += 1
        drivingForce = forceMagnitude * forceDirection

    # Add noise for reducing deadlocks adding naturalness
    if Parameters.noise:
        angle = np.random.uniform(0.0, 1.0) * 2.0 * np.pi
        dist = np.random.uniform(0.0, 1.0) * 0.001
        drivingForce += dist * np.array([np.cos(angle),np.sin(angle),0], dtype='float64')

    # Clamp drivign force, 40 is arbitrary 
    if mag(drivingForce) > 40.0:
        drivingForce = norm(drivingForce) * 40.0
    
    return goalForce + obstacleForce + drivingForce