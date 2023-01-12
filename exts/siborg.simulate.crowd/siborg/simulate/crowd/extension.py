import omni.ext
import omni.ui as ui
import omni.usd
from pxr import Sdf

from . import window
from . import simulator
from .env import Environment

class SFsim(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query
    #  additional information, like where this extension is located on filesystem.

    def on_startup(self, ext_id):
        print("[siborg.simulate.crowd] Social Forces Sim startup")

        self.init_scene()
        self.show() 
        self.rigid_flag = False

    def show(self):
        self._window = ui.Window("Social Forces Demo Settings", width=300, height=220)
        gui_window = window.make_window_elements(self, self._window, self.Sim)

    def create_goal(self):

        omni.kit.commands.execute('CreatePrimWithDefaultXform',
                                    prim_type='Xform',
                                    prim_path='/World/CrowdGoal',
                                    attributes={},
                                    select_new_prim=True)

        omni.kit.commands.execute('MovePrim',
                                    path_from='/World/Xform',
                                    path_to='/World/CrowdGoal',
                                    destructive=False)

        # Code snippet from Mati 
        # subscribe to changes on a prim
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(Sdf.Path('/World/CrowdGoal.xformOp:translate'), 
                                                                      self.Sim.set_xform_goal) 

    def init_scene(self):
        self.World = Environment() 
        self.Sim = simulator.Simulator()
        # Create the goal
        self.create_goal()

    def api_example(self):
        self.Sim._unregister()

        self.Sim = simulator.Simulator(self.World)
        self.demo_api_call(self.Sim)

        # Reset the demo Goal xform watcher to reference the new simulator instance
        self.reset_goal_watcher()
    
    def demo_api_call(self, Sim):
        # Use the builtin function for demo agents
        Sim.rigidbody = self.rigid_flag 
        Sim.init_demo_agents(m=2,n=2,s=1.1)

        if not Sim.rigidbody:
            Sim.create_geompoints() # Create a usdgeom point instance for easy visualization
            Sim.set_geompoints() # update the usdgeom points for visualization
        
        # tell simulator to update positions after each run
        Sim.update_agents_sim = True 
        # tell simulator to handle the update visualization
        Sim.update_viz = True 

        # Register the simulation to updates, and the Sim will handle it from here
        Sim.register_simulation()

    def reset_goal_watcher(self):
        self._goal_subscriber.unsubscribe()

        # subscribe to changes on a prim
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(Sdf.Path('/World/CrowdGoal.xformOp:translate'), 
                                                                      self.Sim.set_xform_goal) 

    def on_shutdown(self):
        print("[siborg.simulate.crowd] Crowd Sim shutdown")
        try: self.Sim._unregister()
        except: pass 
        try: self._goal_subscriber.unsubscribe()
        except: pass

        self._goal_subscriber = None
        self.Sim._simulation_event = None

        self._window = None
        self.Sim = None