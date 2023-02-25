
import omni
from omni.physx.scripts import physicsUtils
from pxr import Gf, UsdPhysics, PhysxSchema, UsdGeom, UsdShade
import usdrt

class Agent:

    def __init__(self):
        stage = omni.usd.get_context().get_stage()

        # Create a sphere representing the agent
        self.skin_mesh , self.skinMeshPath = self.sphere(stage)
        # Set a rigid body material and collider
        self.set_material(stage, self.skinMeshPath)
 
        # Add a translation operator and set it to zero position
        # Since we changed to create this object with an xform, don't need to add, just get it. 
        # self.translateOp = self.skin_mesh.AddTranslateOp()  

        self.translateOp = UsdGeom.XformOp(self.skin_mesh.GetPrim().GetAttribute("xformOp:translate"))
        self.translateOp.Set(Gf.Vec3f(0.0, 0.0, 0.0))

    def sphere(self, stage):

        # Create sphere representing agent
        _, skinMeshPath = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", 
                                                    prim_type="Sphere", 
                                                    prim_path='/World/Agents/Sphere', 
                                                    prepend_default_prim=True)

        skin_mesh = UsdGeom.Mesh.Get(stage, skinMeshPath)
        prim = skin_mesh.GetPrim()

        # setup physics - rigid body
        self.rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(prim)

        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)

        # apply initial velocities
        self.rigidBodyAPI.CreateVelocityAttr().Set(linVelocity)
        self.rigidBodyAPI.CreateAngularVelocityAttr().Set(angularVelocity)

        self.massAPI = UsdPhysics.MassAPI.Apply(prim)
        self.massAPI.CreateMassAttr(2)
        self.massAPI.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        # Add a force attribute
        # shuttleForcePath = skinMeshPath + "/shuttleForce"
        # xform = UsdGeom.Xform.Define(stage, shuttleForcePath)
        # self.forceApi = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())   
        #      
        # self.forceApi = PhysxSchema.PhysxForceAPI.Apply(prim)        
        # self.forceAttr = self.forceApi.GetForceAttr()
        self.usdrt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        prim = self.usdrt_stage.GetPrimAtPath(skinMeshPath)
        self.world_force_attr = prim.CreateAttribute("_worldForce",  usdrt.Sdf.ValueTypeNames.Float3, True)
        
        return skin_mesh, skinMeshPath

    def translate(self, x=0, y=0, z=0):
        self.translateOp.Set(self.translateOp.Get() + Gf.Vec3d( x, y, z))
    
    @property
    def position(self):
        return self.translateOp.Get()

    @property
    def velocity(self):
        return self.rigidBodyAPI.GetVelocityAttr().Get()

    def set_material(self, stage, skinMeshPath):
    
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        # Floor Material
        path = defaultPrimPath + "/rigidMaterial"
        prim_path = stage.GetPrimAtPath(skinMeshPath)

        # Set it as a rigid body 
        rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(prim_path)

        # Add a collider (defaults to mesh triangulation)
        UsdPhysics.CollisionAPI.Apply(prim_path)

        # Apply a specific mass parameter
        UsdPhysics.MassAPI.Apply(prim_path)

        #Get the rigidbody parameter to set values on
        physxRbAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(prim_path)
        #Enable CCD for this object
        physxRbAPI.CreateEnableCCDAttr().Set(True)

        # Create a (separate) physics material that gets added to the object
        path = defaultPrimPath + "/highdensitymaterial"
        UsdShade.Material.Define(stage, path)
        material = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(path))
        material.CreateStaticFrictionAttr().Set(0)
        material.CreateDynamicFrictionAttr().Set(0)
        material.CreateRestitutionAttr().Set(.2)
        material.CreateDensityAttr().Set(0.01)

        # Add material
        physicsUtils.add_physics_material_to_prim(stage, prim_path, path)
