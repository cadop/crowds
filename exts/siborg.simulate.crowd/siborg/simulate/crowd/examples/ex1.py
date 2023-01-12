'''Example for Simulator handling update and using GeomPoints
'''

from siborg.simulate.crowd.simulator import Simulator
import numpy as np
from math import sqrt

def example_1():
    # Example of using API
    Sim = Simulator()
    nagents = 10
    # Some trickery to make a grid of agents and get the cloest number of agents to an even grid
    pos = np.asarray([
                        np.array([(1/2) + (x), (1/2) + (y), 0], dtype=np.double) 
                        for x in range(int(sqrt(nagents))) 
                        for y in range(int(sqrt(nagents)))
                    ])
    pos[:, [2, Sim.world_up]] = pos[:, [Sim.world_up, 2]]
    nagents = len(pos)

    Sim.create_agents(num=nagents, goals=[[10,10,0]], pos=pos) # initialize a set of agents
    Sim.create_geompoints() # Create a usdgeom point instance for easy visualization

    # tell simulator to update positions after each run, if not need to call Sim.integrate()
    Sim.update_agents_sim = True
    # don't have the simulator update the geompoints, we do it ourselves
    Sim.update_viz = True 

    Sim.register_simulation()

example_1()