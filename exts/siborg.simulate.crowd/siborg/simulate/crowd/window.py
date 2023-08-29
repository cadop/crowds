from .models.socialforces import Parameters
import omni.ui as ui

combo_sub = None

def make_window_elements(self, _window, Sim):

    with _window.frame:
        with ui.VStack():

            with ui.HStack():
                ui.Label('Max Speed')
                max_speed = ui.FloatField(height=20)
                max_speed.model.add_value_changed_fn(lambda m : setattr(Parameters, 'max_speed', m.get_value_as_float()))
                max_speed.model.set_value(Parameters.max_speed)
            with ui.HStack():
                ui.Label('Desired Speed')
                v_desired = ui.FloatField(height=20)
                v_desired.model.add_value_changed_fn(lambda m : setattr(Parameters, 'v_desired', m.get_value_as_float()))
                v_desired.model.set_value(Parameters.v_desired)

            with ui.HStack():
                ui.Label('A')
                A = ui.FloatField(height=20)
                A.model.add_value_changed_fn(lambda m : setattr(Parameters, 'A', m.get_value_as_float()))
                A.model.set_value(Parameters.A)

            with ui.HStack():
                ui.Label('B')
                B = ui.FloatField(height=20)
                B.model.add_value_changed_fn(lambda m : setattr(Parameters, 'B', m.get_value_as_float()))
                B.model.set_value(Parameters.B)

            with ui.HStack():
                ui.Label('kn')
                kn = ui.FloatField(height=20)
                kn.model.add_value_changed_fn(lambda m : setattr(Parameters, 'kn', m.get_value_as_float()))
                kn.model.set_value(Parameters.kn)

            with ui.HStack():
                ui.Label('kt')
                kt = ui.FloatField(height=20)
                kt.model.add_value_changed_fn(lambda m : setattr(Parameters, 'kt', m.get_value_as_float()))
                kt.model.set_value(Parameters.kt)

            with ui.HStack():
                ui.Label('Agent grid (nxn)')
                agent_grid = ui.IntField(height=20)
                agent_grid.model.add_value_changed_fn(lambda m : setattr(self, 'grid_size', m.get_value_as_int()))
                agent_grid.model.set_value(3)

            # with ui.HStack():
            #     ui.Label('Agent Mass')
            #     kt = ui.FloatField(height=20)
            #     kt.model.add_value_changed_fn(lambda m : setattr(Sim, 'mass', m.get_value_as_float()))
            #     kt.model.set_value(Sim.mass)

            # with ui.HStack():
            #     ui.Label('Agent Radius')
            #     kt = ui.FloatField(height=20)
            #     kt.model.add_value_changed_fn(lambda m : Sim.set_radius(m.get_value_as_float()))
            #     kt.model.set_value(Sim.radius)

            # with ui.HStack():
            #     ui.Label('Agent Perception Radius')
            #     kt = ui.FloatField(height=20)
            #     kt.model.add_value_changed_fn(lambda m : setattr(Sim, 'perception_radius', m.get_value_as_float()))
            #     kt.model.set_value(Sim.perception_radius)

            # with ui.HStack(height=20):

            #     ui.Button("Gen Agents", clicked_fn=Sim.create_agents)
            #     nagents = ui.IntField(height=5)
            #     nagents.model.set_value(Sim.nagents)
            #     nagents.model.add_value_changed_fn(lambda m : setattr(Sim, 'nagents', m.get_value_as_int()))

            with ui.HStack(height=20):

                ui.Label('GPU', width=20) 
                WarpModel = ui.CheckBox(width=30)
                WarpModel.model.add_value_changed_fn(lambda m : setattr(self, 'gpu_flag', m.get_value_as_bool()))
                WarpModel.model.set_value(True)

                ui.Label('Use Instances', width=20) 
                SFModel = ui.CheckBox(width=30)
                SFModel.model.add_value_changed_fn(lambda m : setattr(self, 'instancer_flag', m.get_value_as_bool()))
                SFModel.model.set_value(True)

                ui.Label('Add Jane', width=5) 
                RigidBody = ui.CheckBox(width=30)
                RigidBody.model.add_value_changed_fn(lambda m : setattr(self, 'jane_flag', m.get_value_as_bool()))
                RigidBody.model.set_value(False)

                ui.Label('Use Direction', width=5) 
                RigidBody = ui.CheckBox(width=30)
                RigidBody.model.add_value_changed_fn(lambda m : setattr(self, 'heading_flag', m.get_value_as_bool()))
                RigidBody.model.set_value(True)

                ui.Label('Rigid Body', width=5) 
                RigidBody = ui.CheckBox(width=30)
                RigidBody.model.add_value_changed_fn(lambda m : setattr(self, 'rigid_flag', m.get_value_as_bool()))
                RigidBody.model.set_value(False)

                ui.Label('PAM', width=20) 
                SFModel = ui.CheckBox(width=30)
                SFModel.model.add_value_changed_fn(lambda m : setattr(self, 'pam_flag', m.get_value_as_bool()))
                SFModel.model.set_value(False)

                # options = ["GeomPoints", "RigidBody"]

                # combo_model: ui.AbstractItemModel = ui.ComboBox(0, *options).model

                # def combo_changed(item_model: ui.AbstractItemModel, item: ui.AbstractItem):
                #     value_model = item_model.get_item_value_model(item)
                #     current_index = value_model.as_int
                #     option = options[current_index]
                #     print(f"Selected '{option}' at index {current_index}.")
                
                # combo_sub = combo_model.subscribe_item_changed_fn(combo_changed)

                # def clicked():
                #     value_model = combo_model.get_item_value_model()
                #     current_index = value_model.as_int
                #     option = options[current_index]
                #     print(f"Button Clicked! Selected '{option}' at index {current_index}.")
                #     self.api_example(current_index)
                # ui.Button("Set Selected Meshes", width=5, clicked_fn=self.assign_meshes)

            ui.Button("Start Demo", width=5, clicked_fn=self.api_example)


            with ui.HStack(height=10):
                pass 