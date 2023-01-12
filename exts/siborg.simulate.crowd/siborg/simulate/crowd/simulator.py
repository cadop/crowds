from math import sqrt
import warp as wp
import numpy as np

from omni.physx import get_physx_interface
import omni
import carb
from pxr import UsdGeom

from siborg.simulate.crowd.agent import Agent

from . import socialforces

class Simulator:

    def __init__(self, world=None):
        self.world = world

        self._goal = [0,0,0]
        self.goals = None

        self.agent_bodies = None
        self.nagents = 25

        # set pereption radius
        self.perception_radius = 3
        # set radius
        self.radius = .5
        # set mass
        self.mass = 2

        self._dt = 1/60.0
        
        self._simulation_event = None

        self.rigidbody = False

        # Tracks if user wants to update agent position on each sim step
        self.update_agents_sim = False 
        self.update_viz = False

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

    def init_demo_agents(self):
        # Initialize agents in a grid for testing
        # randomly set position
        self.agents_pos = np.asarray([np.array([x,x,0], dtype='float64') for x in range(self.nagents)])
        m = int(sqrt(self.nagents))
        n = int(sqrt(self.nagents))
        s = 1
        self.agents_pos = np.asarray([ np.array([(s/2) + (x * s), (s/2) + (y * s), 0]) for x in range(m) for y in range(n)])
        self.nagents = len(self.agents_pos)
        ####

        self.agent_bodies = [Agent() for x in range(self.nagents)]

        for i in range(len(self.agent_bodies)):
            x,y,z = self.agents_pos[i]
            self.agent_bodies[i].translate(x,y,z)

        self.agents_vel = np.asarray([np.array([0,0,0]) for x in range(self.nagents)])

        self.set_radius()
        self.set_mass()
        self.set_perception_radius()

    def create_agents(self, num=None, goals=None, pos=None):
        '''Creates a set of agents and goals

        Uses the class instance defaults for radius, mass, perception, etc.

        Parameters
        ----------
        num : int, optional
            number of agents to create (if not defined in init), by default None
        goals : ndarray([x,y,z]), optional
            either 1 or size equal to number of agents, by default None
        pos : ndarray([x,y,z]), optional
            must be same size as number of agents (otherwise will set all to origin, which is bad),
            because they will explode, by default None
        '''

        # generate n number of agents
        if num:
            self.nagents = num
        # Check we can assign goals to agents
        if not goals:
            goals = [self._goal]
        if len(goals) != 1:
            if len(goals) != self.nagents:
                raise ValueError('If goals is not 1, must be same size as number of agents')
        elif len(goals) == 1:
            self.goals = np.asarray([goals[0] for x in range(self.nagents)], dtype=np.double)
        else: 
            self.goals = goals

        # Set the agent positions
        if pos:
            self.agents_pos = np.asarray(pos, dtype=np.double)
        else:
            self.agents_pos = np.asarray([np.array(0,0,0, dtype=np.double) for x in range(self.nagents)])

        # only create an agent instance if user wants physics-based spheres
        if self.rigidbody:
            self.agent_bodies = [Agent() for x in range(self.nagents)]
            # move agents to their positions
            for i in range(len(self.agent_bodies)):
                x,y,z = self.agents_pos[i]
                self.agent_bodies[i].translate(x,y,z)
        else:
            self.agent_bodies = [None for x in range(self.nagents)]

        # set initial velocities to 0
        self.agents_vel = np.asarray([np.array([0,0,0], dtype=np.double) for x in range(self.nagents)])

        self.set_radius()
        self.set_mass()
        self.set_perception_radius()

    def set_radius(self,v=None):
        '''sets agents radius

        Parameters
        ----------
        v : List[float], float, optional
            set the radius of the agents, if None, all agents get same radius, by default None
        '''

        if v: 
            if type(v) is float:
                self.agents_radi = np.asarray([v for x in range(self.nagents)])
            elif len(v) != self.nagents:
                raise ValueError('Radius array must be same size as number of agents')
            else:
                self.agents_radi = v
        else:
            self.agents_radi = np.asarray([self.radius for x in range(self.nagents)])

    def set_mass(self,v=None):
        '''sets agents mass

        Parameters
        ----------
        v : List[float], optional
            set the mass of the agents, if None, all agents get same mass, by default None

        Raises
        ------
        ValueError
            if size of mass array does not match number of agents
        '''
        if v: 
            if type(v) is float:
                self.agents_mass = np.asarray([v for x in range(self.nagents)])
            elif len(v) != self.nagents:
                raise ValueError('mass array must be same size as number of agents')
            else:
                self.agents_mass = v
        else:
            self.agents_mass = np.asarray([self.mass for x in range(self.nagents)])
        
    def set_perception_radius(self, v=None):
        '''sets agents perception radius

        Parameters
        ----------
        v : List[float], optional
            set the percept radius of the agents, if None, all agents get same raidus, by default None

        Raises
        ------
        ValueError
            if size of perception array does not match number of agents
        '''
        if v: 
            if type(v) is float:
                self.agents_percept = np.asarray([v for x in range(self.nagents)])
            elif len(v) != self.nagents:
                raise ValueError('perception radius array must be same size as number of agents')
            else:
                self.agents_percept = v
        else:
            self.agents_percept = np.asarray([self.perception_radius for x in range(self.nagents)])

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

    def run(self):
        '''Runs the simulation for one step

        Updates agent positions and velocities if instance flag is true

        Returns
        -------
        ndarray[x,y,z] forces
        '''
        force_list = []
        
        for agent in range(self.nagents):
                
            # Get the neighbors of this agent to use in computing forces
            pn = socialforces.get_neighbors(self.agents_pos[agent], 
                                            self.agents_pos, 
                                            self.agents_percept[agent])[1]

            _force = socialforces.compute_force(self.agents_pos[agent], 
                                                self.agents_radi[agent], 
                                                self.agents_vel[agent], 
                                                self.agents_mass[agent], 
                                                self.goals[agent], 
                                                self.agents_pos[pn], 
                                                self.agents_vel[pn], 
                                                self.agents_radi[pn],
                                                self._dt)

            # remove z (up) forces
            _force[2] = 0

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

