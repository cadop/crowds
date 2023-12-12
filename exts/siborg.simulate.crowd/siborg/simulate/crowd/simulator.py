import numpy as np
from numpy import random

from omni.physx import get_physx_interface
import omni
import carb
from pxr import UsdGeom, Gf, Sdf, UsdShade
import warp as wp
import copy
import usdrt

from siborg.simulate.crowd.crowds import CrowdConfig
from siborg.simulate.crowd.models import socialforces
from siborg.simulate.crowd.models import pam

wp.init()
from siborg.simulate.crowd.models import socialforces_warp as crowd_force

class Simulator(CrowdConfig):

    def __init__(self, world=None):
        super().__init__()

        self.world = world

        # a default dt that is surely overwritten later
        self._dt = 1/60.0

        # set radius
        self.radius = 0.7
        self.radius_min = 0.5
        self.radius_max = 1.0

        # A subscription to the physics simulation, used when this class
        # is asked to manage the updates
        self._simulation_event = None

        # Will use a physics scene
        self.rigidbody = False
        self.use_pam = False
        self.on_gpu = False
        self.use_instancer = False
        self.add_jane = False
        self.use_heading = False

        # Tracks if user wants to update agent position on each sim step
        self.update_agents_sim = False 
        self.update_viz = False

        self.instancer_paths = ["/World/PointInstancer_Bob", "/World/PointInstancer_Jane"]
        self.point_instancer_sets = []
        self.agent_instance_path_bob = '/World/Scope/CrowdBob'
        self.agent_instance_path_jane = '/World/Scope/CrowdJane'
        self.instance_forward_vec = (1.0,0.0,0.0) # TODO get from instance object
        self.instance_up_vec = (0.0,1.0,0.0) # TODO Fix to be flexible later
        self.vel_epsilon = 0.05

        self._get_world_up()

    def _get_world_up(self):
        stage = omni.usd.get_context().get_stage()

        up = UsdGeom.GetStageUpAxis(stage)
        
        if up =='X': self.world_up  = 0
        if up =='Y': self.world_up  = 1
        if up =='Z': self.world_up  = 2

        return 

    def register_simulation(self):
        self._callbacks()
        # we need to update the agents, otherwise won't see these results
        self.update_agents_sim = True
        self.update_viz = True

    def _callbacks(self):
        self._simulation_event = get_physx_interface(
                                        ).subscribe_physics_step_events(
                                                                self._on_simulation_update)

    def _unregister(self):
        try:
            self._simulation_event.unsubscribe()
        except:
            self._simulation_event = None

    def _on_simulation_update(self, dt):
        if self.agent_bodies is None:
            return 

        self._dt = dt
        self.run()

    def set_xform_goal(self, p):
        '''set the goal based on a subscribed xform

        Example of usage
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(
                                                Sdf.Path('/World/Goal.xformOp:translate'), 
                                                self.Sim.set_xform_goal) 

        Parameters
        ----------
        p : str(prim path)
            a subscribed path 
        ''' 
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(str(p).split('.')[0])
        goal_point = omni.usd.utils.get_world_transform_matrix(prim).ExtractTranslation()
        # Set agent destination
        self.goals = np.asarray([goal_point for x in range(self.nagents)])
            
    def integrate(self, x, v, f, dt):
        ''' take position, velocity, force, and dt to compute updated position and velocity '''
        v1 = v + ( (f * 1.0) * dt ) # new velocity
        x1 = x + (v1 * dt) # new position

        return x1, v1

    def update_goals(self, new_goal):
        '''update the goals of agents

        Parameters
        ----------
        new_goal : ndarray([x,y,z])
            can either be one goal that is applied to all agents, or a list of 
            the same size as number of agents
        '''
        
        if len(new_goal) == 1:
            self.goals = np.asarray([new_goal for x in range(self.nagents)])
        else:
            self.goals = new_goal

    def compute_step(self, agent):

        # Set the model to PAM if asked
        if self.use_pam: model = pam
        else:            model = socialforces

        # Get the neighbors of this agent to use in computing forces
        pn = model.get_neighbors(self.agents_pos[agent], 
                                        self.agents_pos, 
                                        self.agents_percept[agent])[1] 

        _force = model.compute_force(self.agents_pos[agent], 
                                            self.agents_radi[agent], 
                                            self.agents_vel[agent], 
                                            self.agents_mass[agent], 
                                            self.goals[agent], 
                                            self.agents_pos[pn], 
                                            self.agents_vel[pn], 
                                            self.agents_radi[pn],
                                            self._dt)
        return _force

    def run(self):
        '''Runs the simulation for one step

        Updates agent positions and velocities if instance flag is true

        Returns
        -------
        ndarray[x,y,z] forces
        '''
        self.force_list = []

        for agent in range(self.nagents):

            _force = self.compute_step(agent)

            # remove world (up) forces
            _force[self.world_up] = 0

            # Store all forces to be applied to agents
            self.force_list.append(_force)

        self.step_processing()

    def step_processing(self):
        '''Process the computed step for simulation

        Returns
        -------
        _type_
            _description_
        '''

        # only update agent positions if user requests, otherwise they might want to
        #  update using forces themselves
        if self.update_agents_sim:
            # If using rigid body, apply forces to agents
            if self.rigidbody: 
                self.apply_force(self.force_list)
            else: 
                self.internal_integration()
                if self.use_instancer:
                    self.set_instance_agents()
                else:
                    self.set_geompoints()

    def internal_integration(self):
        # Integrate for new position
        for i in range(self.nagents):
            self.agents_pos[i], self.agents_vel[i] = self.integrate(self.agents_pos[i], 
                                                                    self.agents_vel[i], 
                                                                    self.force_list[i], 
                                                                    self._dt)

    def apply_force(self, force_list):
        '''Used for when rigidbody agents are used

        Parameters
        ----------
        force_list : List[x,y,z]
            list of forces in order of the agents
        '''
        # Apply forces to simulation
        # with Sdf.ChangeBlock():

        # for idx, force in enumerate(force_list):
        #     self._add_force(force, self.agent_bodies[idx], self.agent_bodies[idx].position)
        self._add_force3(force_list, self.agent_bodies)

        # Update positions and velocities 
        for i in range(self.nagents):
            self.agents_pos[i] = self.agent_bodies[i].position
            self.agents_vel[i] = self.agent_bodies[i].velocity

    def _add_force(self, force, rigid_body, position):
        force = carb.Float3(force)
        position = carb.Float3(position)
        get_physx_interface().apply_force_at_pos(rigid_body.skinMeshPath, force, position)  

    def _add_force2(self, force, rigid_body, position):
        # force = Gf.Vec3d(force)
        
        _ = force[0]
        force = Gf.Vec3d(float(force[0]), float(force[1]),float(force[2]))
        rigid_body.forceAttr.Set(force) #position

    def _add_force3(self, force_list, rigid_body):
        # force = Gf.Vec3d(force)
                
        # stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())

        # # prim = stage.GetPrimAtPath("/World/boxActor")
        # attr = prim.CreateAttribute("_worldForce",  usdrt.Sdf.ValueTypeNames.Float3, True)

        # if attr:
        #     attr.Set(usdrt.Gf.Vec3f(50000.0, 0.0, 0.0))

        # prefixes = set(prefix for path in paths for prefix in path.GetPrefixes())
        # with Sdf.ChangeBlock():
            # for path in prefixes:
            #     prim_spec = Sdf.CreatePrimInLayer(layer, path)
            #     prim_spec.specifier = Sdf.SpecifierDef
            #     prim_spec.typeName = UsdGeom.Xform.__name__
        for idx, body in enumerate(rigid_body):
            force = force_list[idx]
            force = usdrt.Gf.Vec3d(float(force[0]), float(force[1]),float(force[2]))
            # body.forceAttr.Set(force) #position
            if body.world_force_attr:
                body.world_force_attr.Set(force)
                    
    def create_geompoints(self, stage_path=None, color=None):
        '''create and manage geompoints representing agents

        Parameters
        ----------
        stage_path : str, optional
            if not set, will use /World/Points, by default None
        color : (r,g,b), optional
            if not set, will make color red, by default None
        '''
        if stage_path: stage_loc = stage_path
        else:          stage_loc = "/World/Points"

        self.stage = omni.usd.get_context().get_stage()
        self.agent_point_prim = UsdGeom.Points.Define(self.stage, stage_loc)
        self.agent_point_prim.CreatePointsAttr()

        width_attr = self.agent_point_prim.CreateWidthsAttr()
        width_attr.Set(self.agents_radi)
        # width_attr.Set([1 for x in range(self.nagents)])

        self.agent_point_prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant
        color_primvar = self.agent_point_prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        
        if color: point_color = color
        else:     point_color = (1,0,0)
        color_primvar.Set([point_color])

    def set_geompoints(self):
        # Set the position with an offset based on the radius
        # Since it is a sphere, we 
        render_pos = np.copy(self.agents_pos)
        render_pos[:,1] += (self.agents_radi/2) 
        self.agent_point_prim.GetPointsAttr().Set(render_pos)

    def create_instance_agents(self):
        
        if self.add_jane:

            bob_size = int(self.nagents/2)
            bob_pos = self.agents_pos[:bob_size]

            point_instancer = self._single_agent_instance(bob_pos, bob_size, self.agent_instance_path_bob, self.instancer_paths[0])
            self.point_instancer_sets.append(point_instancer)

            # TODO find way to split colors of instances 
            jane_size = int(self.nagents/2)
            jane_pos = self.agents_pos[bob_size:]

            point_instancer = self._single_agent_instance(jane_pos, jane_size , self.agent_instance_path_jane, self.instancer_paths[1])
            self.point_instancer_sets.append(point_instancer)

        else:
            point_instancer = self._single_agent_instance(self.agents_pos, self.nagents, self.agent_instance_path_bob, self.instancer_paths[0])
            self.point_instancer_sets.append(point_instancer)


    def _single_agent_instance(self, agent_pos, nagents, agent_instance_path, instance_path):
        stage = omni.usd.get_context().get_stage()

        point_instancer = UsdGeom.PointInstancer.Get(stage, instance_path)
        
        if not point_instancer:
            point_instancer = UsdGeom.PointInstancer(stage.DefinePrim(instance_path, "PointInstancer"))

        point_instancer.CreatePrototypesRel().SetTargets([agent_instance_path])
        self.proto_indices_attr = point_instancer.CreateProtoIndicesAttr()
        self.proto_indices_attr.Set([0] * nagents)

        ## max radius is scale of 1
        agent_scales = self.agents_radi/self.radius_max
        self.agent_instancer_scales = [(x,x,x) for x in agent_scales] # change to numpy 

        # Set scale
        point_instancer.GetScalesAttr().Set(self.agent_instancer_scales)
        point_instancer.GetPositionsAttr().Set(agent_pos)   
        # Set orientation
        rot = Gf.Rotation()
        rot.SetRotateInto(self.instance_forward_vec, self.instance_forward_vec)
        self.agent_headings =  [Gf.Quath(rot.GetQuat()) for x in range(nagents)] 
        point_instancer.GetOrientationsAttr().Set(self.agent_headings)

        return point_instancer
  
    def set_instance_agents(self):
        # update the points 
        # self.point_instancer.CreatePrototypesRel().SetTargets([self.agent_instance_path])
        # self.proto_indices_attr = self.point_instancer.CreateProtoIndicesAttr()
        # self.proto_indices_attr.Set([0] * self.nagents)

        for idx, point_instancer in enumerate(self.point_instancer_sets):
            if len(self.point_instancer_sets) == 1:
                agents_pos = self.agents_pos
            else:
                _slice = int(self.nagents/2)
                if idx == 0:
                    # Positions for this instance
                    agents_pos = self.agents_pos[:_slice]
                else:     
                    # Positions for this instance
                    agents_pos = self.agents_pos[_slice:]

            # Set position
            point_instancer.GetPositionsAttr().Set(agents_pos)   

            if not self.use_heading: continue 
            self.set_heading()

    def set_heading(self):
        for idx, point_instancer in enumerate(self.point_instancer_sets):
            if len(self.point_instancer_sets) == 1:
                agents_vel = self.agents_vel
                nagents = self.nagents
            else:
                _slice = int(self.nagents/2)
                nagents = _slice
                if idx == 0:
                    # Velocities for this instance
                    agents_vel = self.agents_vel[:_slice]
                else:     
                    # Velocities for this instance
                    agents_vel = self.agents_vel[_slice:]

            # Create array of agent headings based on velocity
            normalize_vel = agents_vel
            rot = Gf.Rotation()
            self.agent_headings = []
            
            cur_orient = point_instancer.GetOrientationsAttr().Get()

            for i in range(0, nagents):
                if np.sqrt(normalize_vel[i].dot(normalize_vel[i])) < self.vel_epsilon:
                    tovec = cur_orient[i]
                    self.agent_headings.append(cur_orient[i])
                else:
                    tovec = Gf.Vec3d(tuple(normalize_vel[i]))
                    rot.SetRotateInto(self.instance_forward_vec, tovec)
                    self.agent_headings.append(Gf.Quath(rot.GetQuat()))

            # Set orientation
            point_instancer.GetOrientationsAttr().Set(self.agent_headings)

        return 
    
        #### Change colors

        stage = omni.usd.get_context().get_stage()
        # get path of material
        mat_path = '/CrowdBob/Looks/Linen_Blue'
        linen_mat = Sdf.Path(f'/World/Scope{mat_path}')
        mat_prim = stage.GetPrimAtPath(linen_mat)
        # print(mat_prim)
        # shader_path = '/Shader.inputs:diffuse_tint'
        # tint_shader = f'/World{mat_path}{shader_path}'
        
        shader = omni.usd.get_shader_from_material(mat_prim)
        # print(shader)
        #inp = shader.GetInput('diffuse_tint').Get()
        inp = shader.GetInput('diffuse_tint').Set((0.5,0.5,1.0))


class WarpCrowd(Simulator):
    '''A class to manage the warp-based version of crowd simulation
    '''
    def __init__(self, world=None):
        super().__init__(world)
        self.device = 'cuda:0'

        # generate n number of agents
        self.nagents = 9
        # set radius
        self.radius = 0.7
        self.radius_min = 0.5
        self.radius_max = 1.0
        self.hash_radius = 0.7 # Radius to use for hashgrid
        # set mass
        self.mass = 20
        # set pereption radius
        self.perception_radius = 6
        # self.dt = 1.0/30.0

        self.goal = [0.0,0.0,0.0]
        self.generation_origin = [10,10.0,0.0]

        self.inv_up = wp.vec3(1.0,1.0,1.0) # z-up
        self.inv_up[self.world_up] = 0.0  

        self.on_gpu = True

    def demo_agents(self, s=1.6, m=50, n=50):
        o = self.generation_origin

        # Initialize agents in a grid for testing
        self.agents_pos = np.asarray([
                                      np.array([(s/2) + (x * s) +(o[0]/2) ,
                                                (s/2) + (y * s) +(o[1]/2),
                                                 0
                                                ], dtype=np.double) 
                                      for x in range(m) 
                                      for y in range(n)
                                    ])
        self.nagents = len(self.agents_pos)
        self.configure_params()

    def configure_params(self):
        '''Convert all parameters to warp
        '''
        self.agents_pos = np.asarray(self.agents_pos)
        # self.agents_pos = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.nagents)])
        self.agents_vel = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.nagents)])

        # # Set a quath for heading
        # rot = Gf.Rotation()
        # rot.SetRotateInto(self.instance_forward_vec, self.instance_forward_vec) # from, to
        # _hquat = Gf.Quath(rot.GetQuat())
        # # Get rotation between agent forward direction

        self.agents_hdir = np.asarray([np.array([0,0,0,1], dtype=float) for x in range(self.nagents)])

        self.force_list = np.asarray([np.array([0,0,0], dtype=float) for x in range(self.nagents)])
        self.agents_radi = np.random.uniform(self.radius_min, self.radius_max, self.nagents)
        self.agents_mass = [self.mass for x in range(self.nagents)]
        self.agents_percept = np.asarray([self.perception_radius for x in range(self.nagents)])
        self.agents_goal = np.asarray([np.array(self.goal, dtype=float) for x in range(self.nagents)])

        self.agent_force_wp = wp.zeros(shape=self.nagents,device=self.device, dtype=wp.vec3)
        self.agents_pos_wp = wp.array(self.agents_pos, device=self.device, dtype=wp.vec3)
        self.agents_vel_wp = wp.array(self.agents_vel, device=self.device, dtype=wp.vec3)

        self.agents_hdir_wp = wp.array(self.agents_hdir, device=self.device, dtype=wp.vec4)

        self.agents_goal_wp = wp.array(self.agents_goal, device=self.device, dtype=wp.vec3)
        self.agents_radi_wp = wp.array(self.agents_radi, device=self.device, dtype=float)
        self.agents_mass_wp = wp.array(self.agents_mass, device=self.device, dtype=float)
        self.agents_percept_wp = wp.array(self.agents_percept, device=self.device, dtype=float)

        self.xnew_wp = wp.zeros_like(wp.array(self.agents_pos, device=self.device, dtype=wp.vec3))
        self.vnew_wp = wp.zeros_like(wp.array(self.agents_pos, device=self.device, dtype=wp.vec3))

        self.hdir_wp = wp.zeros_like(wp.array(self.agents_hdir, device=self.device, dtype=wp.vec4))

    def config_hasgrid(self, nagents=None):
        '''Create a hash grid based on the number of agents
            Currently assumes z up

        Parameters
        ----------
        nagents : int, optional
            _description_, by default None
        '''

        if nagents is None: nagents = self.nagents
        self.grid = wp.HashGrid(dim_x=200, dim_y=200, dim_z=1, device=self.device)
        # self.grid = wp.HashGrid(dim_x=nagents, dim_y=nagents, dim_z=1, device=self.device)

    def config_mesh(self, points, faces):
        '''Create a warp mesh object from points and faces

        Parameters
        ----------
        points : List[[x,y,z]]
            A list of floating point xyz vertices of a mesh
        faces : List[int]
            A list of integers corresponding to vertices. Must be triangle-based
        '''
        # fake some points and faces if empty list was passed 
        if len(points) == 0:
            points = [(0,0,0), (0,0,0), (0,0,0)]
            faces = [[1, 2, 3]]

        # print(points)
        # print(faces)

        # Init mesh for environment collision
        self.mesh = wp.Mesh( points=wp.array(points, dtype=wp.vec3, device=self.device),
                            indices=wp.array(faces, dtype=int ,device=self.device)
                            )

    def update_goals(self, new_goal):
        if len(new_goal) == 1:
            self.goals = np.asarray([new_goal for x in range(self.nagents)])
        else:
            self.goals = new_goal
        self.agents_goal_wp = wp.array(self.goals, device=self.device, dtype=wp.vec3)

    def run(self):
        # Rebuild hashgrid given new positions
        self.grid.build(points=self.agents_pos_wp, radius=self.hash_radius)

        # launch kernel
        wp.launch(kernel=crowd_force.get_forces,
                dim=self.nagents,
                inputs=[self.agents_pos_wp, self.agents_vel_wp, self.agents_goal_wp, self.agents_radi_wp, 
                        self.agents_mass_wp, self._dt, self.agents_percept_wp, self.grid.id, self.mesh.id,
                        self.inv_up],
                outputs=[self.agent_force_wp],
                device=self.device
                )
        self.force_list = self.agent_force_wp.numpy()

        self.step_processing()

        self.agents_pos_wp = wp.array(self.agents_pos, device=self.device, dtype=wp.vec3)
        self.agents_vel_wp = wp.array(self.agents_vel, device=self.device, dtype=wp.vec3)

        return self.agent_force_wp

    def internal_integration(self):
        # Given the forces, integrate for pos and vel
        wp.launch(kernel=crowd_force.integrate,
                dim=self.nagents,
                inputs=[self.agents_pos_wp, self.agents_vel_wp, self.agent_force_wp, self._dt],
                outputs=[self.xnew_wp, self.vnew_wp],
                device=self.device
                )

        self.agents_pos_wp = self.xnew_wp
        self.agents_vel_wp = self.vnew_wp

        self.agents_pos = self.agents_pos_wp.numpy()
        self.agents_vel = self.agents_vel_wp.numpy()

    def set_heading(self):

        up = wp.vec3(0.0,1.0,0.0)
        forward = wp.vec3(1.0,0.0,0.0)

        wp.launch(kernel=crowd_force.heading,
                dim=self.nagents,
                inputs=[self.agents_vel_wp, up, forward],
                outputs=[self.hdir_wp],
                device=self.device
                )

        self.agents_hdir_wp = self.hdir_wp
        self.agents_hdir = self.agents_hdir_wp.numpy()

        for idx, point_instancer in enumerate(self.point_instancer_sets):
            if len(self.point_instancer_sets) == 1:
                agent_headings = self.agents_hdir
            else:
                _slice = int(self.nagents/2)
                if idx == 0:
                    agent_headings = self.agents_hdir[:_slice]
                else:     
                    agent_headings = self.agents_hdir[_slice:]
            # Set orientation
            point_instancer.GetOrientationsAttr().Set(agent_headings)
