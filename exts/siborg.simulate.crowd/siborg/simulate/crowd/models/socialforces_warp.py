import warp as wp

Tau = wp.constant(0.5) # s (acceleration)
A = wp.constant(2000.0) # N
B = wp.constant(0.08) # m
kn = wp.constant(1.2 * 100000) # kg/s^-2
kt = wp.constant(2.4 * 100000) # kg/m^-1 s^-2

max_speed = wp.constant(10.0) # m/s
v_desired = wp.constant(2.5) # m/s


@wp.kernel
def get_forces(positions: wp.array(dtype=wp.vec3),
                velocities: wp.array(dtype=wp.vec3),
                goals: wp.array(dtype=wp.vec3),
                radius: wp.array(dtype=float),
                mass: wp.array(dtype=float),
                dt: float,
                percept : wp.array(dtype=float),
                grid : wp.uint64,
                mesh: wp.uint64,
                inv_up: wp.vec3,
                forces: wp.array(dtype=wp.vec3),
                ):

    # thread index
    tid = wp.tid()

    cur_pos = positions[tid]
    cur_rad = radius[tid]
    cur_vel = velocities[tid]
    cur_mass = mass[tid]
    goal = goals[tid]
    pn = percept[tid]

    _force = compute_force(cur_pos,
                                cur_rad,
                                cur_vel,
                                cur_mass,
                                goal,
                                positions,
                                velocities,
                                radius,
                                dt,
                                pn,
                                grid,
                                mesh)

    # Clear any vertical forces with Element-wise mul
    _force = wp.cw_mul(_force, inv_up)

    # compute distance of each point from origin
    forces[tid] = _force

@wp.kernel
def integrate(x : wp.array(dtype=wp.vec3),
                v : wp.array(dtype=wp.vec3),
                f : wp.array(dtype=wp.vec3),
                dt: float,
                xnew: wp.array(dtype=wp.vec3),
                vnew: wp.array(dtype=wp.vec3), 
            ):
    
    tid = wp.tid()

    x0 = x[tid]
    v0 = v[tid]
    f0 = f[tid]

    v1 = v0 + (f0*1.0) * dt
    x1 = x0 + v1 * dt

    xnew[tid] = x1
    vnew[tid] = v1


@wp.kernel
def heading(v : wp.array(dtype=wp.vec3),
            up : wp.vec3, 
            forward : wp.vec3,
            hdir: wp.array(dtype=wp.vec4), 
            ):
    
    tid = wp.tid()
    v0 = v[tid]
    vnorm = wp.normalize(v0)

    hdir[tid] = velocity_to_quaternion(up, forward, vnorm)


@wp.func
def velocity_to_quaternion(up : wp.vec3, 
                           forward : wp.vec3, 
                           velocity: wp.vec3):
    # Construct a quaternion that rotates the agent's forward direction to align with the velocity vector
    if wp.length(forward) > 0: forward = wp.normalize(forward)
    if wp.length(velocity) > 0: velocity = wp.normalize(velocity)
    else: 
        velocity = forward

    dot = wp.dot(forward, velocity) # Clip the dot product to avoid numerical instability
    if dot == 1.0:
        # If the forward and velocity vectors are already aligned, return the identity quaternion
        return wp.vec4(0.0, 0.0, 0.0, 1.0)
    else:
        axis = wp.cross(forward, velocity)
        axis = up * wp.sign(wp.dot(axis, up))  # Project the axis onto the up plane
        if wp.length(axis) > 0.0: axis = wp.normalize(axis)  # Normalize the axis of rotation
        else:axis = up  # Use a default axis of rotation if the iwput is a zero vector
        angle = wp.acos(dot)  # Calculate the angle of rotation with clipping
        
        qw = wp.cos(angle/2.0)  # Calculate the scalar component of the quaternion
        qx = wp.sin(angle/2.0) * axis[0]  # Calculate the vector component of the quaternion
        qy = wp.sin(angle/2.0) * axis[1]  # Calculate the vector component of the quaternion
        qz = wp.sin(angle/2.0) * axis[2]  # Calculate the vector component of the quaternion
        
        return wp.vec4(qx, qy, qz, qw)


@wp.func
def calc_goal_force(goal: wp.vec3, 
                    pos: wp.vec3, 
                    vel: wp.vec3, 
                    mass: float, 
                    v_desired: float, 
                    dt: float):
    ee_i = wp.normalize(goal - pos)
    force = mass * ( ( (v_desired * ee_i) - vel ) / (Tau) )

    return force 

@wp.func
def calc_wall_force(rr_i: wp.vec3,
                    ri: float,
                    vv_i: wp.vec3,
                    mesh: wp.uint64):
    '''
    rr_i : position
    ri : radius
    vv_i : velocity
    Computes: (A * exp[(ri-diw)/B] + kn*g(ri-diw))*niw - kt * g(ri-diw)(vi * tiw)tiw
    '''

    face_index = int(0)
    face_u = float(0.0)
    face_v = float(0.0)
    sign = float(0.0)

    force = wp.vec3(0.0,0.0,0.0)
    # Define the up direction 
    up_dir = wp.vec3(0.0, 0.0, 1.0)

    max_dist = float(ri * 5.0)

    has_point = wp.mesh_query_point(mesh, rr_i, max_dist, sign, face_index, face_u, face_v)

    if (not has_point):
        return wp.vec3(0.0, 0.0, 0.0)
        
    p = wp.mesh_eval_position(mesh, face_index, face_u, face_v)

    # d_iw = distance to wall W
    d_iw = wp.length(p - rr_i)

    # vector of the wall to the agent
    nn_iw = wp.normalize(rr_i - p)
    # perpendicular vector of the agent-wall (tangent force)
    tt_iw = wp.cross(up_dir, nn_iw)
    if wp.dot(vv_i, tt_iw) < 0.0: 
        tt_iw = -1.0 * tt_iw

    # Compute force
    #  f_iW = { A * exp[(ri-diw)/B] + kn*g(ri-diw) } * niw 
    #   - kt * g(ri-diw)(vi * tiw)tiw
    f_rep = ( A * wp.exp((ri-d_iw)/B) + kn * G(ri, d_iw) ) * nn_iw 
    f_tan = kt * G(ri,d_iw) * wp.dot(vv_i, tt_iw) * tt_iw
    force = f_rep - f_tan

    return force 

@wp.func
def calc_agent_force(rr_i: wp.vec3, 
                     ri: float, 
                     vv_i: wp.vec3, 
                     pn_rr: wp.array(dtype=wp.vec3), 
                     pn_vv: wp.array(dtype=wp.vec3), 
                     pn_r: wp.array(dtype=float),
                     pn: float,
                     grid : wp.uint64,
                     ):
    '''Sum the forces of neighboring agents'''

    #  Set the total force of the other agents to zero
    force = wp.vec3(0.0, 0.0, 0.0)
    ff_ij = wp.vec3(0.0, 0.0, 0.0)
    rr_j = wp.vec3(0.0, 0.0, 0.0)

    # create grid query around point
    query = wp.hash_grid_query(grid, rr_i, pn)
    index = int(0)

    #  Iterate through the neighbors and sum (f_ij)
    while(wp.hash_grid_query_next(query, index)):
        j = index
        neighbor = pn_rr[j]

        # compute distance to neighbor point
        dist = wp.length(rr_i-neighbor)
        if (dist <= pn):
            #  Get position and velocity of neighbor agent
            rr_j = pn_rr[j]
            vv_j = pn_vv[j]
            #  Get radii of neighbor agent
            rj = pn_r[j]

            #  Pass agent position to AgentForce calculation
            ff_ij = neighbor_force(rr_i, ri, vv_i, rr_j, rj, vv_j)

            #  Sum Forces
            force += ff_ij
    
    return force

@wp.func
def neighbor_force(rr_i: wp.vec3, 
                ri: float, 
                vv_i: wp.vec3, 
                rr_j: wp.vec3, 
                rj: float, 
                vv_j: wp.vec3):
    '''Calculate the force exerted by another agent.
    Take in this agent (i) and a neighbors (j) position and radius'''

    #  Sum of radii
    rij = ri + rj
    #  distance between center of mass
    d_ij = wp.length(rr_i - rr_j)

    #  "n_ij is the normalized vector points from pedestrian j to i"
    n_ij = wp.normalize(rr_i - rr_j) # Normalized vector pointing from j to i

    #  t_ij "Vector of tangential relative velocity pointing from i to j." 
    #  A sliding force is applied on agent i in this direction to reduce the relative velocity.
    t_ij = vv_j - vv_i
    dv_ji = wp.dot(vv_j - vv_i, t_ij)

    #  Calculate f_ij
    force =  repulsion(rij, d_ij, n_ij) + proximity(rij, d_ij, n_ij) + sliding(rij, d_ij, dv_ji, t_ij)

    return force

@wp.func
def G(r_ij: float, 
      d_ij: float
      ):
    # g(x) is a function that returns zero if pedestrians touch
    # otherwise is equal to the argument x 
    if (d_ij > r_ij): return 0.0
    return r_ij - d_ij

@wp.func
def repulsion(r_ij: float, 
              d_ij: float,
              n_ij: wp.vec3):
    force = A * wp.exp( (r_ij - d_ij) / B) * n_ij
    return force

@wp.func
def proximity(r_ij: float, 
              d_ij: float,
              n_ij: wp.vec3):
    force = (kn * G(r_ij, d_ij)) * n_ij # body force
    return force 

@wp.func
def sliding(r_ij: float, 
            d_ij: float, 
            dv_ji: float, 
            t_ij: wp.vec3):
    force = kt * G(r_ij, d_ij) * (dv_ji * t_ij)
    return force

@wp.func
def compute_force(rr_i: wp.vec3, 
                    ri: float,
                    vv_i: wp.vec3, 
                    mass:float,
                    goal:wp.vec3, 
                    pn_rr: wp.array(dtype=wp.vec3),
                    pn_vv: wp.array(dtype=wp.vec3),
                    pn_r: wp.array(dtype=float), 
                    dt: float,
                    pn: float,
                    grid : wp.uint64,
                    mesh: wp.uint64
                    ):
    ''' 
    rr_i : position
    ri : radius
    vv_i : velocity
    pn_rr : List[perceived neighbor positions]
    pn_vv : List[perceived neighbor velocities]
    pn_r : List[perceived neighbor radius]
    '''

    # Get the force for this agent to the goal
    goal = calc_goal_force(goal, rr_i, vv_i, mass, v_desired, dt)
    agent = calc_agent_force(rr_i, ri, vv_i, pn_rr, pn_vv, pn_r, pn, grid)
    wall = calc_wall_force(rr_i, ri, vv_i, mesh)
    # Sum of forces
    force = goal + agent + wall

    force = wp.normalize(force) * wp.min(wp.length(force), max_speed)

    return force


