import numpy as np
from siborg.simulate.crowd.agent import Agent

class CrowdConfig:
    
    def __init__(self):

        self._goal = [0,0,0]
        self.goals = None

        self.agent_bodies = None
        self.nagents = 1

        # set pereption radius
        self.perception_radius = 1.5
        # set radius
        self.radius = .5
        # set mass
        self.mass = 2
        
        # Will use a physics scene
        self.rigidbody = False

        # Assume z-up world
        self.world_up = 2 

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
        if pos is not None:
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

    def init_demo_agents(self, m=5, n=5, s=1, o=[0,0,0]):
        '''Create a set of demo agents

        Parameters
        ----------
        m : int, optional
            number of agents in row, by default 5
        n : int, optional
            number of agents in col, by default 5
        s : int, optional
            spacing between agents, by default 1
        '''
        o = self.generation_origin
        # Initialize agents in a grid for testing
        self.agents_pos = np.asarray([
                                      np.array([(s/2) + (x * s) +(o[0]/2) ,
                                                (s/2) + (y * s) +(o[1]/2),
                                                 0],
                                                dtype=np.double)
                                      for x in range(m) 
                                      for y in range(n)
                                    ])
        
        # # Initialize agents in a grid for testing
        # self.agents_pos = np.asarray([
        #                               np.array([(s/2) + (x * s), (s/2) + (y * s), 0], dtype=np.double) 
        #                               for x in range(m) 
        #                               for y in range(n)
        #                             ])

        self.agents_pos[:, [2, self.world_up]] = self.agents_pos[:, [self.world_up, 2]]

        self.nagents = len(self.agents_pos)

        ####
        if self.rigidbody:
            self.agent_bodies = [Agent() for x in range(self.nagents)]
            for i in range(len(self.agent_bodies)):
                x,y,z = self.agents_pos[i]
                self.agent_bodies[i].translate(x,y,z)
        else:
            self.agent_bodies = [None for x in range(self.nagents)]

        self.goals = np.asarray([self._goal for x in range(self.nagents)], dtype=np.double)

        self.agents_vel = np.asarray([np.array([0,0,0],dtype=np.double) for x in range(self.nagents)])

        self.set_radius()
        self.set_mass()
        self.set_perception_radius()
