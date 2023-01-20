import numpy as np
from pxr import UsdGeom, Gf, Usd
import omni

def get_mesh(usd_stage, objs):

    points, faces = [],[]

    for obj in objs:
        f_offset = len(points)
        f, p = convert_to_mesh(usd_stage.GetPrimAtPath(obj))
        points.extend(p)
        faces.extend(f+f_offset)

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
