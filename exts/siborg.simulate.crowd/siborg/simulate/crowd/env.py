import omni
import omni.kit.commands
from pxr import Usd, Gf
from pxr import UsdGeom
from pxr import UsdPhysics, PhysxSchema
class Environment:
    
    def __init__(self):
        print('Initializing Environment')

        self._stage = omni.usd.get_context().get_stage()
        self.set_scene(self._stage)

    def set_scene(self, stage):
        print(f'Setting up {stage}')
        self._stage = stage
        self.defaultPrimPath = str(self._stage.GetDefaultPrim().GetPath())

        # Physics scene
        # UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        self.scene = UsdPhysics.Scene.Define(stage, self.defaultPrimPath + "/physicsScene")
        stage_axis = UsdGeom.GetStageUpAxis(stage)
        gravity_dir = Gf.Vec3f(0.0, 0.0, 0)

        if stage_axis is 'X': gravity_dir[0] = -1.0
        if stage_axis is 'Y': gravity_dir[1] = -1.0
        if stage_axis is 'Z': gravity_dir[2] = -1.0

        self.scene.CreateGravityDirectionAttr().Set(gravity_dir)
        self.scene.CreateGravityMagnitudeAttr().Set(9.81)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(self.scene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)

        # Check if there is a physics groundplane in the scene
        plane_path = self.defaultPrimPath+"/GroundPlane"

        if self._stage.GetPrimAtPath(plane_path).IsValid():
            pass
        else:
            # If not, make one
            omni.kit.commands.execute('AddGroundPlaneCommand',
                                        stage=self._stage,
                                        planePath='/GroundPlane',
                                        axis=UsdGeom.GetStageUpAxis(stage),
                                        size=1.0,
                                        position=Gf.Vec3f(0.0, 0.0, 0.0),
                                        color=Gf.Vec3f(0.5, 0.5, 0.5))



