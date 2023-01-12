import omni
from pxr import UsdGeom, Gf
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
        # self.scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self.scene.CreateGravityMagnitudeAttr().Set(9.81)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(self.scene.GetPrim())
        physxSceneAPI.CreateEnableCCDAttr().Set(True)


