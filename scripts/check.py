"""出来上がった .blend を検査する：道路が地面に埋もれていないか、白線が車道に埋もれていないか

使い方: Blender -b <bandobashi_3dmap.blend> --python check.py
"""
import bpy
from mathutils import Vector
from mathutils.bvhtree import BVHTree


def bvh(name):
    ob = bpy.data.objects[name]
    me = ob.data
    return BVHTree.FromPolygons([v.co.copy() for v in me.vertices], [tuple(p.vertices) for p in me.polygons])


def below(tree, p):
    hit = tree.ray_cast(Vector((p.x, p.y, p.z + 0.5)), Vector((0, 0, -1)), 3.0)
    return hit[0].z if hit[0] is not None else None


def check(obj, ref, need, step=7):
    """obj の面の中心と辺の中点で、真下の ref との高さの差を測る"""
    tree = bvh(ref)
    me = bpy.data.objects[obj].data
    vs = me.vertices
    bad = total = 0
    worst = 0.0
    BAD = []
    for p in list(me.polygons)[::step]:
        idx = list(p.vertices)
        pts = [p.center] + [(vs[idx[k]].co + vs[idx[(k + 1) % len(idx)]].co) / 2 for k in range(len(idx))]
        for q in pts:
            z = below(tree, q)
            if z is None:
                continue
            total += 1
            gap = q.z - z
            if gap < need:
                bad += 1
                worst = min(worst, gap)
                BAD.append((gap, round(q.x), round(q.y), round(q.z, 1)))
    print(f"[検査] {obj} と {ref}: {total} 点中 {bad} 点 ({bad / max(total, 1) * 100:.2f}%) が {need}m 未満の差"
          f" / 最悪 {worst:.2f}m", flush=True)
    BAD.sort()
    print("   悪い所（差, x, y, z）:", BAD[:8])
    import collections
    print("   200m区画ごと:", collections.Counter((b[1] // 200 * 200, b[2] // 200 * 200) for b in BAD).most_common(6))


check("road_surface", "terrain", 0.02)
check("road_sidewalk", "terrain", 0.02)
check("road_footpath", "terrain", 0.02)
check("marking_lane_white", "road_surface", 0.02, step=3)
check("marking_crosswalk", "road_surface", 0.02, step=3)
