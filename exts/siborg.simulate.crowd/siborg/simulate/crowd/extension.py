import numpy as np

import omni.ext
import omni.ui as ui
import omni.usd
from omni.physx import get_physx_interface

from . import window
from . import simulator
from .env import Environment
 
class SFsim(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query
    #  additional information, like where this extension is located on filesystem.

    def on_startup(self, ext_id):
        print("[siborg.simulate.crowd] Social Forces Sim startup")

        self.goal_prim_path = '/World/CrowdGoals'
        self.grid_size = 3

        self.init_scene()
        self.show() 
        self.rigid_flag = False
        self.pam_flag = False


        self.goal_prim_dict = {} # {Prim path, subscriber}

    def show(self):
        self._window = ui.Window("Social Forces Demo Settings", width=300, height=220)
        gui_window = window.make_window_elements(self, self._window, self.Sim)

    def init_goal_prim(self, prim_path):
        omni.kit.commands.execute('CreatePrimWithDefaultXform',
                                    prim_type='Xform',
                                    prim_path=prim_path,
                                    attributes={},
                                    select_new_prim=True)

    def modify_goals(self, _new_goals):
        if len(_new_goals) == 0: return 
        # Assign goals based on number of goals available
        if len(_new_goals)>self.Sim.nagents: 
            _new_goals = _new_goals[self.Sim.nagents:]
        # Get strides
        goal_cast = np.array_split(self.Sim.goals, len(_new_goals))
        # Reassign the split arrays their new goals
        for idx in range(len(goal_cast)):
            goal_cast[idx][:] = _new_goals[idx]

        # Reshape into xyz vector
        goal_cast = np.asarray(goal_cast)
        goal_cast = np.vstack(goal_cast)

        # Update the simulations goals
        self.Sim.update_goals(goal_cast)

    def init_scene(self):
        self.World = Environment() 
        self.Sim = simulator.Simulator()
        # Create the goal hierarchy
        self.init_goal_prim(self.goal_prim_path)

        self._on_update_sub = get_physx_interface().subscribe_physics_step_events(self._on_update_event)

    def _on_update_event(self, dt):
        # Check the Goals xform path and see if there are any changes needed to the goal watchers
        self.stage = omni.usd.get_context().get_stage()

        parent_prim =  self.stage.GetPrimAtPath(self.goal_prim_path)
        children = parent_prim.GetAllChildren()

        # Check if any children are gone from our dict, if so, unsubscribe their watcher
        dead_kids = [kid for kid in self.goal_prim_dict.keys() if kid not in children]

        for kid in dead_kids: 
            try: self.goal_prim_dict[kid].unsubscribe()
            except: self.goal_prim_dict[kid] = None
            self.goal_prim_dict.pop(kid)

        # Check if there are any new children not in our dict, if so, add them as a goal and update watcher
        babies = [child for child in children if child not in self.goal_prim_dict.keys()]
        for baby in babies: 
            self.goal_prim_dict[baby] = None

        # Update the goals
        new_goals = []
        for x in self.goal_prim_dict.keys():
            _prim = x
            t = omni.usd.utils.get_world_transform_matrix(_prim).ExtractTranslation()
            new_goals.append(t)

        new_goals = np.asarray(new_goals)
        if len(new_goals) == 0: new_goals = np.asarray([0,0,0], dtype=float)
        self.modify_goals(new_goals)

    def api_example(self):
        self.Sim._unregister()

        self.Sim = simulator.Simulator(self.World)
        self.demo_api_call(self.Sim)
    
    def demo_api_call(self, Sim):
        # Use the builtin function for demo agents
        Sim.rigidbody = self.rigid_flag 
        Sim.init_demo_agents(m=self.grid_size,n=self.grid_size,s=1.1)

        if not Sim.rigidbody:
            Sim.create_geompoints() # Create a usdgeom point instance for easy visualization
            Sim.set_geompoints() # update the usdgeom points for visualization
        if self.pam_flag:
            Sim.use_pam = True
        # tell simulator to update positions after each run
        Sim.update_agents_sim = True 
        # tell simulator to handle the update visualization
        Sim.update_viz = True 

        # Register the simulation to updates, and the Sim will handle it from here
        Sim.register_simulation()

    def on_shutdown(self):
        print("[siborg.simulate.crowd] Crowd Sim shutdown")
        try: self.Sim._unregister()
        except: pass 
        try: self._goal_subscriber.unsubscribe()
        except: self._goal_subscriber = None

        try: self._on_update_sub.unsubscribe()
        except: self._on_update_sub = None 

        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None