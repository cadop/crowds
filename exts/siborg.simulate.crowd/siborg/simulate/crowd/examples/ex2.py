'''Example for Simulator handling update and using GeomPoints.
Uses a helper function for initializing agents
'''

from siborg.simulate.crowd.simulator import Simulator

def example_2():
    Sim = Simulator()
    # Use a builtin helper function to generate a grid of agents
    Sim.init_demo_agents(m=3,n=5,s=1.1)

    Sim.create_geompoints() # Create a usdgeom point instance for easy visualization
    # tell simulator to update positions after each run, if not need to call Sim.integrate()
    Sim.update_agents_sim = True
    # don't have the simulator update the geompoints, we do it ourselves
    Sim.update_viz = True 

    Sim.register_simulation()

example_2()