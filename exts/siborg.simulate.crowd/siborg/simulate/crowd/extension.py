import omni.ext
import omni.ui as ui
import omni.usd
from pxr import Sdf
from enum import Enum

from . import window
from . import simulator as physx_sf
from .env import Environment


class SFsim(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.

    def on_startup(self, ext_id):
        print("[siborg.simulate.crowd] Social Forces Sim startup")

        self._count = 0

        self.init_scene()
        self.show() 

        # Code snippet from Mati 
        # subscribe to changes on a prim
        watcher = omni.usd.get_watcher()
        self._goal_subscriber = watcher.subscribe_to_change_info_path(Sdf.Path('/World/Goal1.xformOp:translate'), self.Sim.set_goal) 

        watcher2 = omni.usd.get_watcher()
        self._goal_subscriber2 = watcher2.subscribe_to_change_info_path(Sdf.Path('/World/Goal2.xformOp:translate'), self.Sim.set_goal2)

    def show(self):
        self._window = ui.Window("Social Forces Settings", width=300, height=300)
        gui_window = window.make_window_elements(self._window, self.Sim)

    def unsubscribe(self):
        self.Sim._simulation_event = None
        self._goal_subscriber = None
        self._goal_subscriber2 = None

    def on_shutdown(self):
        print("[siborg.simulate.crowd] Social Forces Sim shutdown")

        self.unsubscribe()

        try: 
            del self.World._update_sub
        except: pass
        self._window = None
        self.Sim = None

    def init_scene(self):
        self.World = Environment() 
        # Defaults to the physx simulator
        self.Sim = physx_sf.Simulator(self.World)