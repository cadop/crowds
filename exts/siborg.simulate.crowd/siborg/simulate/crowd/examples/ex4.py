'''_summary_
'''

from siborg.simulate.crowd.simulator import Simulator

def example_4():
    # Example of using API
    Sim = Simulator()
    Sim.rigidbody = True # use rigid bodies
    Sim.init_demo_agents(m=3, n=5, s=1.1)

    # Register the simulation to updates, and the Sim will handle it from here
    Sim.register_simulation()

    # tell simulator to update positions after each run, if not need to call Sim.integrate()
    Sim.update_agents_sim = True 
    # tell simulator to handle the update visualization
    Sim.update_viz = True 

example_4()