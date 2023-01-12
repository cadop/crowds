import omni.ext
import omni.ui as ui
import omni.usd
from pxr import Sdf
from enum import Enum

import numpy as np

from . import window
from . import simulator as physx_sf
from .env import Environment
from math import sqrt

from omni.physx import get_physx_interface
class SFsim(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        print("[siborg.simulate.crowd] Social Forces Sim startup")

        self.init_scene()
        self.show() 

        self._api_sim_updater = False # true for using the simulation class for updating, false for us doing it

        # Code snippet from Mati 
        # subscribe to changes on a prim
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(Sdf.Path('/World/Goal.xformOp:translate'), 
                                                                      self.Sim.set_xform_goal) 

    def show(self):
        self._window = ui.Window("Social Forces Settings", width=300, height=400)
        gui_window = window.make_window_elements(self, self._window, self.Sim)

    def _unsubscribe(self):
        try: self.Sim._unregister()
        except: pass 
        try: self._goal_subscriber.unsubscribe()
        except: pass
        try: self._simulation_event.unsubscribe()
        except: pass 

        self._goal_subscriber = None
        self.Sim._simulation_event = None
        self._simulation_event = None

    def on_shutdown(self):
        print("[siborg.simulate.crowd] Social Forces Sim shutdown")

        self._unsubscribe()

        try: 
            del self.World._update_sub
        except: pass
        self._window = None
        self.Sim = None

    def init_scene(self):
        self.World = Environment() 
        # Defaults to the physx simulator
        self.Sim = physx_sf.Simulator()

    def reset_goal_watcher(self):
        self._goal_subscriber.unsubscribe()

        # subscribe to changes on a prim
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(Sdf.Path('/World/Goal.xformOp:translate'), 
                                                                      self.Sim.set_xform_goal) 

    def api_example(self, v):
        self.Sim._unregister()
        self.registration_unsubscribe()

        if v == 0:
            self.Sim = physx_sf.Simulator()
            self.api_usage_callback(self.Sim) # Should not have an update call that runs simulator
            self._api_sim_updater = True

        elif v == 1:
            self.Sim = physx_sf.Simulator()
            self.api_usage(self.Sim) # Needs to be updated by us
            self.sim_registration()
            self._api_sim_updater = False

        elif v == 2:
            self.Sim = physx_sf.Simulator(self.World)
            self.api_usage_rigid(self.Sim) # Needs to be updated by us
            self.sim_registration()
            self._api_sim_updater = False

        elif v == 3:
            self.Sim = physx_sf.Simulator(self.World)
            self.api_usage_rigid_callback(self.Sim) # Should not have an update call that runs simulator
            self._api_sim_updater = True 

        # Reset the demo Goal xform watcher
        self.reset_goal_watcher()

    def sim_registration(self):
        self._simulation_event = get_physx_interface().subscribe_physics_step_events(self._on_update)

    def registration_unsubscribe(self):
        try:
            self._simulation_event.unsubscribe()
        except:
            pass
        self._simulation_event = None

    def _on_update(self, dt):
        # Run one step of simulation
        self.Sim._dt = dt
        forces = self.Sim.run()

        if not self._api_sim_updater:
            ##  You can put this in an on_update event
            if self.Sim.rigidbody:
                # for rigid body we need to apply the forces instead of set_geompoints
                self.Sim.apply_force(forces)
            else:
                self.Sim.set_geompoints() # update the usdgeom points for visualization

    def api_usage(self, Sim):
        # Example of using API
        # Sim = physx_sf.Simulator()
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
        Sim.update_viz = False 

        # ##  You can put this in an on_update event
        # # Run one step of simulation
        # forces = Sim.run() # don't need to use forces since we told simulator to update
        
        # This will only happen once just so we can see stuff
        Sim.set_geompoints() # update the usdgeom points for visualization

    def api_usage_callback(self, Sim):
        # Example of using API
        # Sim = physx_sf.Simulator()
        # nagents = 10
        # # Some trickery to make a grid of agents and get the cloest number of agents to an even grid
        # pos = [ [(1/2) + (x * 1), (1/2) + (y * 1), 0] for x in range(int(sqrt(nagents))) for y in range(int(sqrt(nagents)))]
        # nagents = len(pos)
        # Sim.create_agents(num=nagents, goals=[[10,10,0]], pos=pos) # initialize a set of agents

        Sim.init_demo_agents()

        Sim.create_geompoints() # Create a usdgeom point instance for easy visualization
        Sim.set_geompoints() # update the usdgeom points for visualization
        
        # tell simulator to update positions after each run, if not need to call Sim.integrate()
        Sim.update_agents_sim = True 
        # tell simulator to handle the update visualization
        Sim.update_viz = True 

        # Register the simulation to updates, and the Sim will handle it from here
        Sim.register_simulation()

        # You will need to unregister at the end with
        # Sim._simulation_event.unsubscribe()

    def api_usage_rigid(self, Sim):
        # Example of using API
        # Sim = physx_sf.Simulator()
        Sim.rigidbody = True # use rigid bodies
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

        # tell simulator to update positions after each run, if not need to call Sim.integrate()
        Sim.update_agents_sim = True
        # don't have the simulator update the geompoints, we do it ourselves
        Sim.update_viz = False 

        ##  You can put this in an on_update event
        # # Run one step of simulation
        # Sim.run()

    def api_usage_rigid_callback(self, Sim):
        # Example of using API
        # Sim = physx_sf.Simulator()
        Sim.rigidbody = True # use rigid bodies
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
        # Register the simulation to updates, and the Sim will handle it from here
        Sim.register_simulation()

        # tell simulator to update positions after each run, if not need to call Sim.integrate()
        Sim.update_agents_sim = True 
        # tell simulator to handle the update visualization
        Sim.update_viz = True 

        # You will need to unregister at the end with
        # Sim._simulation_event.unsubscribe()
