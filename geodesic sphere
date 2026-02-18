import bpy
import bmesh
import math
from mathutils import Vector

# ============================================================
# CONNECTED Geodesic Hex/Pent Lattice Sphere (No Floating Bits)
# Strategy:
#   1) Build Icosphere (triangulated)
#   2) Build DUAL mesh (hex/pent topology, connected)
#   3) Wireframe modifier on the dual => connected strut network
#   4) Boolean INTERSECT with outer sphere volume => spherical boundary
#   5) Boolean DIFFERENCE with inner sphere => hollow shell
# Optional: Voxel Remesh to fuse any near-touching struts (super robust)
# ============================================================

# -------------------------
# Parameters (mm-like, if you export with 1 BU = 1 mm)
# -------------------------
sphere_radius   = 60.0
subdivisions    = 4

strut_thickness = 4.5
wall_thickness  = 1.5

make_dome       = False

use_voxel_remesh = True
voxel_size       = 0.5

# --- NEW: organic randomness controls ---
seed = 7
organic_jitter_strength = 3.0   # in Blender units (mm-like if 1BU=1mm). Try 1.5–6.0
organic_smooth_iters    = 10    # smoothing iterations of the jitter field (higher = more "veiny" flow)
organic_smooth_lambda   = 0.55  # 0..1 smoothing amount

delete_helpers   = True
import random

def project_to_radius(p: Vector, radius: float):
    if p.length == 0:
        return Vector((radius, 0, 0))
    return p.normalized() * radius

def organic_jitter_dual_on_sphere(dual_obj, radius, strength=2.0, smooth_iters=8, lam=0.5, seed=0):
    """
    Moves vertices tangentially in a smooth, organic way, then reprojects to the sphere.
    Preserves connectivity because topology doesn't change.
    """
    random.seed(seed)

    me = dual_obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.verts.ensure_lookup_table()

    # Build adjacency list
    nbrs = {v: [e.other_vert(v) for e in v.link_edges] for v in bm.verts}

    # Initial random tangent offsets
    offsets = {}
    for v in bm.verts:
        n = v.co.normalized()
        t, b = tangent_basis(n)
        # random direction in tangent plane
        ang = random.uniform(0.0, 2.0 * math.pi)
        dir_tan = (math.cos(ang) * t + math.sin(ang) * b)
        mag = random.uniform(0.2, 1.0) * strength
        offsets[v] = dir_tan.normalized() * mag

    # Smooth the offset field so it becomes organic rather than "salt & pepper"
    for _ in range(max(0, smooth_iters)):
        new_offsets = {}
        for v in bm.verts:
            acc = Vector((0,0,0))
            ln = 0
            for u in nbrs[v]:
                acc += offsets[u]
                ln += 1
            if ln > 0:
                avg = acc / ln
                new_offsets[v] = offsets[v].lerp(avg, lam)
            else:
                new_offsets[v] = offsets[v]
        offsets = new_offsets

    # Apply offsets tangentially + reproject to sphere
    for v in bm.verts:
        n = v.co.normalized()
        off = offsets[v]
        # remove any normal component (keep purely tangential)
        off = off - n * off.dot(n)
        v.co = project_to_radius(v.co + off, radius)

    bm.to_mesh(me)
    bm.free()
    me.update()

# -------------------------
# Helpers
# -------------------------
def deselect_all():
    for o in bpy.context.selected_objects:
        o.select_set(False)

def set_active(obj):
    bpy.context.view_layer.objects.active = obj

def apply_modifier(obj, mod_name):
    deselect_all()
    obj.select_set(True)
    set_active(obj)
    bpy.ops.object.modifier_apply(modifier=mod_name)

def delete_all_objects():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

def tangent_basis(n: Vector):
    n = n.normalized()
    a = Vector((1,0,0)) if abs(n.x) < 0.9 else Vector((0,1,0))
    t = n.cross(a).normalized()
    b = n.cross(t).normalized()
    return t, b

def build_dual_from_object(obj, name="Dual"):
    # Build dual of obj's mesh (assumes triangulated surface)
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    # ensure triangulated
    bmesh.ops.triangulate(bm, faces=bm.faces[:], quad_method='BEAUTY', ngon_method='BEAUTY')
    bm.faces.ensure_lookup_table()
    bm.verts.ensure_lookup_table()

    dual_bm = bmesh.new()

    face_to_dv = {}
    for f in bm.faces:
        c = f.calc_center_median()
        dv = dual_bm.verts.new(c)
        face_to_dv[f] = dv
    dual_bm.verts.ensure_lookup_table()

    # faces around each original vertex, sorted by angle
    for v in bm.verts:
        linked = list(v.link_faces)
        if len(linked) < 3:
            continue

        n = v.co.normalized()
        t, b = tangent_basis(n)

        items = []
        for f in linked:
            c = f.calc_center_median()
            u = (c - v.co)
            u = u - n * u.dot(n)
            if u.length < 1e-9:
                ang = 0.0
            else:
                u.normalize()
                ang = math.atan2(u.dot(b), u.dot(t))
            items.append((ang, f))

        items.sort(key=lambda x: x[0])
        dverts = [face_to_dv[f] for _, f in items]

        try:
            dual_bm.faces.new(dverts)
        except ValueError:
            pass

    dual_bm.faces.ensure_lookup_table()
    me = bpy.data.meshes.new(name)
    dual_bm.to_mesh(me)
    dual_bm.free()
    bm.free()

    dual_obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(dual_obj)
    return dual_obj

# -------------------------
# Start clean
# -------------------------
delete_all_objects()

# -------------------------
# 1) Base sphere volume (for intersect + hollow)
# -------------------------
bpy.ops.mesh.primitive_ico_sphere_add(radius=sphere_radius, subdivisions=subdivisions, location=(0,0,0))
base = bpy.context.active_object
base.name = "BaseSphere"

# We'll keep a copy as the boolean "outer volume"
outer = base.copy()
outer.data = base.data.copy()
outer.name = "OuterVolume"
bpy.context.collection.objects.link(outer)

# -------------------------
# 2) Build dual mesh (connected graph)
# -------------------------
dual = build_dual_from_object(base, name="DualGeodesic")
dual.location = (0,0,0)
organic_jitter_dual_on_sphere(
    dual, sphere_radius,
    strength=organic_jitter_strength,
    smooth_iters=organic_smooth_iters,
    lam=organic_smooth_lambda,
    seed=seed
)
# -------------------------
# 3) Turn dual into connected strut network (Wireframe)
# -------------------------
struts = dual.copy()
struts.data = dual.data.copy()
struts.name = "StrutNetwork"
bpy.context.collection.objects.link(struts)

wf = struts.modifiers.new("Wireframe", "WIREFRAME")
wf.thickness = strut_thickness
wf.use_replace = True        # replace faces with wireframe struts
wf.use_boundary = True
wf.use_even_offset = True
apply_modifier(struts, wf.name)

# Optional: fuse any near-touching details into one watertight mesh (very robust for printing)
if use_voxel_remesh:
    rm = struts.modifiers.new("VoxelRemesh", "REMESH")
    rm.mode = 'VOXEL'
    rm.voxel_size = voxel_size
    rm.use_smooth_shade = False
    apply_modifier(struts, rm.name)

# -------------------------
# 4) Keep struts spherical: INTERSECT with outer sphere volume
# -------------------------
bool_int = struts.modifiers.new("IntersectOuter", "BOOLEAN")
bool_int.operation = 'INTERSECT'
bool_int.solver = 'EXACT'
bool_int.object = outer
apply_modifier(struts, bool_int.name)

# -------------------------
# 5) Hollow it: subtract inner sphere
# -------------------------
inner_radius = max(0.01, sphere_radius - wall_thickness)

bpy.ops.mesh.primitive_ico_sphere_add(radius=inner_radius, subdivisions=subdivisions, location=(0,0,0))
inner = bpy.context.active_object
inner.name = "InnerVolume"

bool_hollow = struts.modifiers.new("Hollow", "BOOLEAN")
bool_hollow.operation = 'DIFFERENCE'
bool_hollow.solver = 'EXACT'
bool_hollow.object = inner
apply_modifier(struts, bool_hollow.name)

# -------------------------
# 6) Optional: cut to dome (top half)
# -------------------------
if make_dome:
    # Use a big cube as a cutter to keep Z>=0
    bpy.ops.mesh.primitive_cube_add(size=sphere_radius*4, location=(0,0,sphere_radius*2))
    keep_cube = bpy.context.active_object
    keep_cube.name = "DomeKeepCube"

    dome_bool = struts.modifiers.new("MakeDome", "BOOLEAN")
    dome_bool.operation = 'INTERSECT'
    dome_bool.solver = 'EXACT'
    dome_bool.object = keep_cube
    apply_modifier(struts, dome_bool.name)

    if delete_helpers:
        bpy.data.objects.remove(keep_cube, do_unlink=True)

# -------------------------
# 7) Cleanup helpers
# -------------------------
if delete_helpers:
    bpy.data.objects.remove(base, do_unlink=True)
    bpy.data.objects.remove(dual, do_unlink=True)
    bpy.data.objects.remove(outer, do_unlink=True)
    bpy.data.objects.remove(inner, do_unlink=True)

# Shade smooth for preview (optional)
deselect_all()
struts.select_set(True)
set_active(struts)
bpy.ops.object.shade_smooth()

print("Done: Connected geodesic lattice (no floaters).")
