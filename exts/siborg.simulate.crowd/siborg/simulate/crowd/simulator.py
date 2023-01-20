import numpy as np

from omni.physx import get_physx_interface
import omni
import carb
from pxr import UsdGeom

from siborg.simulate.crowd.crowds import CrowdConfig
from siborg.simulate.crowd.models import socialforces
from siborg.simulate.crowd.models import pam


import warp as wp
wp.init()

from siborg.simulate.crowd.models import socialforces_warp as crowd_force
class Simulator(CrowdConfig):

    def __init__(self, world=None):
        super().__init__()

        self.world = world

        # a default dt that is surely overwritten later
        self._dt = 1/60.0
        
        # A subscription to the physics simulation, used when this class
        # is asked to manage the updates
        self._simulation_event = None

        # Will use a physics scene
        self.rigidbody = False
        self.use_pam = False

        # Tracks if user wants to update agent position on each sim step
        self.update_agents_sim = False 
        self.update_viz = False

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
        force_list = []

        for agent in range(self.nagents):

            _force = self.compute_step(agent)

            # remove z (up) forces
            _force[self.world_up] = 0

            # Store all forces to be applied to agents
            force_list.append(_force)

        # only update agent positions if user requests, otherwise they might want to
        #  update using forces themselves
        if self.update_agents_sim:
            # If using rigid body, apply forces to agents
            if self.rigidbody: 
                self.apply_force(force_list)
                return force_list

            # Integrate for new position
            for i in range(self.nagents):
                self.agents_pos[i], self.agents_vel[i] = self.integrate(self.agents_pos[i], 
                                                                        self.agents_vel[i], 
                                                                        force_list[i], 
                                                                        self._dt)
            if self.update_viz: 
                self.set_geompoints()

        return force_list

    def apply_force(self, force_list):
        '''Used for when rigidbody agents are used

        Parameters
        ----------
        force_list : List[x,y,z]
            list of forces in order of the agents
        '''
        # Apply forces to simulation
        for idx, force in enumerate(force_list):
            self._add_force(force, self.agent_bodies[idx].skinMeshPath, self.agent_bodies[idx].position)

        # Update positions and velocities 
        for i in range(self.nagents):
            self.agents_pos[i] = self.agent_bodies[i].position
            self.agents_vel[i] = self.agent_bodies[i].velocity

    def _add_force(self, force, rigid_body, position):
        force = carb.Float3(force)
        position = carb.Float3(position)
        get_physx_interface().apply_force_at_pos(rigid_body, force, position)        

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
        width_attr.Set([1 for x in range(self.nagents)])

        self.agent_point_prim.CreateDisplayColorAttr()
        # For RTX renderers, this only works for UsdGeom.Tokens.constant
        color_primvar = self.agent_point_prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
        
        if color: point_color = color
        else:     point_color = (1,0,0)
        color_primvar.Set([point_color])

    def set_geompoints(self):
        self.agent_point_prim.GetPointsAttr().Set(self.agents_pos)


class WarpCrowd(Simulator):
    '''A class to manage the warp-based version of crowd simulation
    '''
    def __init__(self, world=None):
        super().__init__(world)
        self.device = 'cuda'

        # generate n number of agents
        self.nagents = 9
        # set radius
        self.radius = 0.7
        self.radius_min = 0.5
        self.radius_max = 1.0
        self.hash_radius = 0.7 # Radius to use for hashgrid
        # set mass
        self.mass = 80
        # set pereption radius
        self.perception_radius = 6
        self.dt = 1.0/30.0

        self.goal = [0.0,0.0,0.0]

        # self.demo_agents()
        self.configure_params()
        self.params_to_warp()

    def demo_agents(self, s=1.1, m=50, n=50):
        # Initialize agents in a grid for testing
        self.agents_pos = np.asarray([
                                      np.array([(s/2) + (x * s), (s/2) + (y * s), 0], dtype=np.double) 
                                      for x in range(m) 
                                      for y in range(n)
                                    ])
        self.nagents = len(self.agents_pos)

    def configure_params(self):
        self.agents_pos = np.asarray([np.array([0,0,0]) for x in range(self.nagents)])
        self.agents_vel = np.asarray([np.array([0,0,0]) for x in range(self.nagents)])
        self.agents_radi = np.random.uniform(self.radius_min, self.radius_max, self.nagents)
        self.agents_mass = [self.mass for x in range(self.nagents)]
        self.agents_percept = np.asarray([self.perception_radius for x in range(self.nagents)])
        self.agents_goal = np.asarray([np.array(self.goal, dtype=float) for x in range(self.nagents)])

    def params_to_warp(self):
        '''Convert all parameters to warp
        '''
        self.agent_force_wp = wp.zeros(shape=self.nagents,device=self.device, dtype=wp.vec3)
        self.agents_pos_wp = wp.array(self.agents_pos, device=self.device, dtype=wp.vec3)
        self.agents_vel_wp = wp.array(self.agents_vel, device=self.device, dtype=wp.vec3)
        self.agents_goal_wp = wp.array(self.agents_goal, device=self.device, dtype=wp.vec3)
        self.agents_radi_wp = wp.array(self.agents_radi, device=self.device, dtype=float)
        self.agents_mass_wp = wp.array(self.agents_mass, device=self.device, dtype=float)
        self.agents_percept_wp = wp.array(self.agents_percept, device=self.device, dtype=float)

        self.xnew_wp = wp.zeros_like(wp.array(self.agents_pos, device=self.device, dtype=wp.vec3))
        self.vnew_wp = wp.zeros_like(wp.array(self.agents_pos, device=self.device, dtype=wp.vec3))

    def config_hasgrid(self, nagents=None):
        '''Create a hash grid based on the number of agents
            Currently assumes z up

        Parameters
        ----------
        nagents : int, optional
            _description_, by default None
        '''

        if nagents is None: nagents = self.nagents
        self.grid = wp.HashGrid(dim_x=nagents, dim_y=nagents, dim_z=1, device=self.device)

    def config_mesh(self, points, faces):
        '''Create a warp mesh object from points and faces

        Parameters
        ----------
        points : List[[x,y,z]]
            A list of floating point xyz vertices of a mesh
        faces : List[int]
            A list of integers corresponding to vertices. Must be triangle-based
        '''
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

    # def compute_step(self):
    def run(self):
        # Rebuild hashgrid given new positions
        self.grid.build(points=self.agents_pos_wp, radius=self.hash_radius)

        # launch kernel
        wp.launch(kernel=crowd_force.get_forces,
                dim=self.nagents,
                inputs=[self.agents_pos_wp, self.agents_vel_wp, self.agents_goal_wp, self.agents_radi_wp, 
                        self.agents_mass_wp, self.dt, self.agents_percept_wp, self.grid.id, self.mesh.id],
                outputs=[self.agent_force_wp],
                device=self.device
                )

        # Given the forces, integrate for pos and vel
        wp.launch(kernel=crowd_force.integrate,
                dim=self.nagents,
                inputs=[self.agents_pos_wp, self.agents_vel_wp, self.agent_force_wp, self.dt],
                outputs=[self.xnew_wp, self.vnew_wp],
                device=self.device
                )
    
        self.agents_pos_wp = self.xnew_wp
        self.agents_vel_wp = self.vnew_wp

        self.agents_pos = self.agents_pos_wp.numpy()

        self.set_geompoints()

        return self.agent_force_wp
