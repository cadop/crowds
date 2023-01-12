'''_summary_
'''

import time
from omni.physx import get_physx_interface
from siborg.simulate.crowd.simulator import Simulator

Sim = Simulator()
start_time = time.time()
_simulation_event = None

def example_3():
    # Example of using API
    # Use a builtin helper function to generate a grid of agents
    Sim.init_demo_agents(m=3, n=5, s=1.1)

    Sim.create_geompoints() # Create a usdgeom point instance for easy visualization
    Sim.set_geompoints() # update the usdgeom points for visualization

    # tell simulator to update positions after each run, if not need to call Sim.integrate()
    Sim.update_agents_sim = True
    # don't have the simulator update the geompoints, we do it ourselves
    Sim.update_viz = False 

    # Register to our own physx update
    sim_subscriber()

def sim_subscriber():
    # This would need to get cleaned up
    _simulation_event = get_physx_interface().subscribe_physics_step_events(_on_update)

def _on_update(dt):
    # Run one step of simulation
    # don't need to use forces since we told simulator to update
    forces = Sim.run() 
    Sim.set_geompoints() # update the usdgeom points for visualization

    # For this demo we will unsubscribe after a few seconds
    if time.time() - start_time > 100 :
        print('ending')
        _simulation_event.unsubscribe()

example_3()

