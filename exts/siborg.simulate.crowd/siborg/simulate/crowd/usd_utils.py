import numpy as np
from pxr import UsdGeom, Gf, Usd
import omni

def get_mesh(usd_stage, objs):

    points, faces = [],[]

    for obj in objs:
        f_offset = len(points)
        # f, p = convert_to_mesh(obj)#usd_stage.GetPrimAtPath(obj))
        f, p = meshconvert(obj)#usd_stage.GetPrimAtPath(obj))
        
        points.extend(p)
        faces.extend(f+f_offset)

    return points, faces

def get_all_stage_mesh(stage, start_prim):

    found_meshes = []

    # Traverse the scene graph and print the paths of prims, including instance proxies
    for x in Usd.PrimRange(start_prim, Usd.TraverseInstanceProxies()):
        if x.IsA(UsdGeom.Mesh):
            found_meshes.append(x)

    points, faces = get_mesh(stage, found_meshes)
   
    return points, faces


def convert_to_mesh(prim):
    ''' convert a prim to BVH '''

    # Get mesh name (prim name)
    m = UsdGeom.Mesh(prim)

    # Get verts and triangles
    tris = m.GetFaceVertexIndicesAttr().Get()

    tris_cnt = m.GetFaceVertexCountsAttr().Get()

    verts = m.GetPointsAttr().Get()

    tri_list = np.array(tris)
    vert_list = np.array(verts)

    xform = UsdGeom.Xformable(prim)
    time = Usd.TimeCode.Default() # The time at which we compute the bounding box
    world_transform: Gf.Matrix4d = xform.ComputeLocalToWorldTransform(time)
    translation: Gf.Vec3d = world_transform.ExtractTranslation()
    rotation: Gf.Rotation = world_transform.ExtractRotationMatrix()
    # rotation: Gf.Rotation = world_transform.ExtractRotation()
    scale: Gf.Vec3d = Gf.Vec3d(*(v.GetLength() for v in world_transform.ExtractRotationMatrix()))
    rotation = rotation.GetOrthonormalized()

    # New vertices
    vert_list = np.dot((vert_list * scale ), rotation) + translation

    # vert_scaled = vert_list
    # vert_list[:,0] *= scale[0]
    # vert_list[:,1] *= scale[1]
    # vert_list[:,2] *= scale[2]
    # vert_rotated = np.dot(vert_scaled, rotation) # Rotate points
    # vert_translated = vert_rotated + translation
    # vert_list = vert_translated


    # Check if the face counts are 4, if so, reshape and turn to triangles
    if tris_cnt[0] == 4:
        quad_list = tri_list.reshape(-1,4)
        tri_list = quad_to_tri(quad_list)
        tri_list = tri_list.flatten()

    return tri_list, vert_list

def quad_to_tri(a):
    idx = np.flatnonzero(a[:,-1] == 0)
    out0 = np.empty((a.shape[0],2,3),dtype=a.dtype)      

    out0[:,0,1:] = a[:,1:-1]
    out0[:,1,1:] = a[:,2:]

    out0[...,0] = a[:,0,None]

    out0.shape = (-1,3)

    mask = np.ones(out0.shape[0],dtype=bool)
    mask[idx*2+1] = 0
    return out0[mask]

def selected_as_mesh():
    # Get the current active selection of the stage
    stage = omni.usd.get_context().get_stage()

    # Get the selections from the stage
    _usd_context = omni.usd.get_context()
    _selection = _usd_context.get_selection()
    selected_paths = _selection.get_selected_prim_paths()
    # Expects a list, so take first selection
    prims = [stage.GetPrimAtPath(x) for x in selected_paths]

    points, faces = get_mesh(stage, selected_paths)
    return points, faces

def children_as_mesh(stage, parent_prim):
    children = parent_prim.GetAllChildren()
    children = [child.GetPrimPath() for child in children]
    points, faces = get_mesh(stage, children)
    return points, faces



def meshconvert(prim):

    # Create an XformCache object to efficiently compute world transforms
    xform_cache = UsdGeom.XformCache()

    # Get the mesh schema
    mesh = UsdGeom.Mesh(prim)
    
    # Get verts and triangles
    tris = mesh.GetFaceVertexIndicesAttr().Get()
    if not tris:
        return [], []
    tris_cnt = mesh.GetFaceVertexCountsAttr().Get()

    # Get the vertices in local space
    points_attr = mesh.GetPointsAttr()
    local_points = points_attr.Get()
    
    # Convert the VtVec3fArray to a NumPy array
    points_np = np.array(local_points, dtype=np.float64)
    
    # Add a fourth component (with value 1.0) to make the points homogeneous
    num_points = len(local_points)
    ones = np.ones((num_points, 1), dtype=np.float64)
    points_np = np.hstack((points_np, ones))

    # Compute the world transform for this prim
    world_transform = xform_cache.GetLocalToWorldTransform(prim)

    # Convert the GfMatrix to a NumPy array
    matrix_np = np.array(world_transform, dtype=np.float64).reshape((4, 4))

    # Transform all vertices to world space using matrix multiplication
    world_points = np.dot(points_np, matrix_np)

    tri_list = convert_to_triangle_mesh(tris, tris_cnt)
    tri_list = tri_list.flatten()

    world_points = world_points[:,:3]

    return tri_list, world_points

def convert_to_triangle_mesh(FaceVertexIndices, FaceVertexCounts):
    """
    Convert a list of vertices and a list of faces into a triangle mesh.
    
    A list of triangle faces, where each face is a list of indices of the vertices that form the face.
    """
    
    # Parse the face vertex indices into individual face lists based on the face vertex counts.

    faces = []
    start = 0
    for count in FaceVertexCounts:
        end = start + count
        face = FaceVertexIndices[start:end]
        faces.append(face)
        start = end

    # Convert all faces to triangles
    triangle_faces = []
    for face in faces:
        if len(face) < 3:
            newface = []  # Invalid face
        elif len(face) == 3:
            newface = [face]  # Already a triangle
        else:
            # Fan triangulation: pick the first vertex and connect it to all other vertices
            v0 = face[0]
            newface = [[v0, face[i], face[i + 1]] for i in range(1, len(face) - 1)]

        triangle_faces.extend(newface)
    
    return np.array(triangle_faces)




# from pxr import UsdGeom, Sdf, Usd
# import os 

# def add_ext_reference(prim: Usd.Prim, ref_asset_path: str, ref_target_path: Sdf.Path) -> None:
#     references: Usd.References = prim.GetReferences()
#     references.AddReference(
#         assetPath=ref_asset_path,
#         primPath=ref_target_path # OPTIONAL: Reference a specific target prim. Otherwise, uses the referenced layer's defaultPrim.
#     )
# class makescope:

#     def __init__(self):
#         self.stage = omni.usd.get_context().get_stage()
#         scope = UsdGeom.Scope.Define(self.stage, Sdf.Path('/World/Scope'))
#         ref_prim = UsdGeom.Xform.Define(self.stage, Sdf.Path('/World/Scope/CrowdJane')).GetPrim()
#         dir_path = os.path.join('G:/ProjectRepos/crowds/exts/siborg.simulate.crowd/siborg/simulate/crowd/data/', 'CrowdBob.usda')
#         add_ext_reference(ref_prim, dir_path, Sdf.Path("<Default Prim>"))
        
# ms = makescope()


