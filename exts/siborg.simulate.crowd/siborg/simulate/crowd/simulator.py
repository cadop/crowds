from math import sqrt
import warp as wp
import numpy as np

from omni.physx import get_physx_interface
import omni
import carb

from siborg.simulate.crowd.agent import Agent

from . import socialforces

class Simulator:

    def __init__(self, world=None):
        self.world = world

        self.goal = [0,100,0]
        self.goal2 = [100,0,0]

        self.agent_bodies = None
        self.nagents = 25

        # set pereption radius
        self.perception_radius = 4
        # set radius
        self.radius = .5
        # set mass
        self.mass = 2
        
        self._simulation_event = None

        self._callbacks()

    def _callbacks(self):
        self._simulation_event = get_physx_interface().subscribe_physics_step_events(
                                                                self._on_simulation_update)

    def _on_simulation_update(self, dt):
        if self.agent_bodies is None:
            return 

        self._dt = dt
        self.run()

    def create_agents(self):
        # generate n number of agents
        nagents = self.nagents

        # randomly set position
        self.agents_pos = np.asarray([np.array([x,x,0], dtype='float64') for x in range(nagents)])

        m = int(sqrt(nagents))
        n = int(sqrt(nagents))

        s = 1

        self.agents_pos = np.asarray([ np.array([(s/2) + (x * s), (s/2) + (y * s), 0]) for x in range(m) for y in range(n)])

        self.nagents = len(self.agents_pos)
        nagents = self.nagents

        self.agent_bodies = [Agent() for x in range(nagents)]

        for i in range(len(self.agent_bodies)):
            x,y,z = self.agents_pos[i]
            self.agent_bodies[i].translate(x,y,z)

        self.agents_vel = np.asarray([np.array([0,0,0]) for x in range(nagents)])

        self.set_radius()
        self.set_mass()
        self.set_perception_radius()

    def set_radius(self,v=None):
        if v: self.radius = v
        self.agents_radi = np.asarray([self.radius for x in range(self.nagents)])

    def set_mass(self,v=None):
        if v: self.mass = v
        self.agents_mass = [self.mass for x in range(self.nagents)]
        
    def set_perception_radius(self, v=None):
        if v: self.perception_radius = v
        self.agents_percept = np.asarray([self.perception_radius for x in range(self.nagents)])

    def set_goal(self, p):
        stage = omni.usd.get_context().get_stage()

        prim = stage.GetPrimAtPath(str(p).split('.')[0])
        goal_point = omni.usd.utils.get_world_transform_matrix(prim).ExtractTranslation()

        # Set agent destination
        self.goal =  goal_point

    def set_goal2(self, p):
        stage = omni.usd.get_context().get_stage()

        prim = stage.GetPrimAtPath(str(p).split('.')[0])
        goal_point = omni.usd.utils.get_world_transform_matrix(prim).ExtractTranslation()

        # Set agent destination
        self.goal2 =  goal_point
            
    def run(self):

        nagents = self.nagents
        goal = self.goal

        agents_pos = self.agents_pos
        agents_radi = self.agents_radi
        agents_vel = self.agents_vel
        agents_mass = self.agents_mass
        agents_percept = self.agents_percept

        force_list = []
        
        dt = self._dt

        for agent in range(nagents):

            # Change the goal for half the agents
            if agent%2 == 0:
                goal = self.goal
            else:
                goal = self.goal2
                
            cur_pos = agents_pos[agent]
            cur_rad = agents_radi[agent]
            cur_vel = agents_vel[agent]
            cur_mass = agents_mass[agent]

            # Get the neighbors of this agent to use in computing forces
            pn = socialforces.get_neighbors(cur_pos, agents_pos, agents_percept[agent])[1]

            _force = socialforces.compute_force(cur_pos, cur_rad, cur_vel, 
                                        cur_mass, goal, 
                                        agents_pos[pn], agents_vel[pn], agents_radi[pn],
                                        dt)

            # remove z (up) forces
            _force[2] = 0

            # Store all forces to be applied to agents
            force_list.append(_force)
            # force_list.append(_force*0.1)

        # Apply forces to simulation
        for idx, force in enumerate(force_list):
            self.apply_force(force, self.agent_bodies[idx].skinMeshPath, self.agent_bodies[idx].position)

        # Update positions and velocities 
        for i in range(nagents):
            self.agents_pos[i] = self.agent_bodies[i].position
            self.agents_vel[i] = self.agent_bodies[i].velocity

    def apply_force(self, force, rigid_body, position):
        force = carb.Float3(force)
        position = carb.Float3(position)
        get_physx_interface().apply_force_at_pos(rigid_body, force, position)        

