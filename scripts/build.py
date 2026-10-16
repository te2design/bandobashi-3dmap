"""阪東橋 3km 四方の 3D マップを Blender で組み立てて GLB に書き出す

使い方:
  Blender -b --factory-startup --python build.py -- <osm.json> <dem.json> <buildings.jsonl> <出力フォルダ>

座標: Blender 上は x=東, y=北, z=標高(東京湾平均海面からの m)。原点 = 阪東橋駅。
      GLB(three.js) では X=東, Y=上, Z=南 になる（書き出し時に自動変換）。
"""
import bpy
import bmesh  # noqa: F401
import json
import math
import os
import sys
import heapq
from bisect import bisect_left
from mathutils.geometry import tessellate_polygon

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import to_local, HALF, LAT0, LON0  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:]
OSM_PATH, DEM_PATH, BLDG_PATH, OUT = argv[:4]
os.makedirs(OUT, exist_ok=True)
LOG = []


def log(*a):
    s = " ".join(str(x) for x in a)
    print(s, flush=True)
    LOG.append(s)


# ---------------------------------------------------------------- 高さ（地形）
TSTEP = 10.0  # 地形メッシュの間隔 [m]
dem = json.load(open(DEM_PATH))
_k = int(round(TSTEP / dem["step"]))
N = (dem["n"] - 1) // _k + 1
TH0 = [[dem["h"][(j * _k) * dem["n"] + i * _k] for i in range(N)] for j in range(N)]
TH = [row[:] for row in TH0]  # 川を掘り下げた後の地形（表示用）


def _H(T, x, y):
    fx = min(max((x + HALF) / TSTEP, 0.0), N - 1 - 1e-9)
    fy = min(max((y + HALF) / TSTEP, 0.0), N - 1 - 1e-9)
    i, j = int(fx), int(fy)
    u, v = fx - i, fy - j
    if u + v <= 1:
        h00 = T[j][i]
        return h00 + (T[j][i + 1] - h00) * u + (T[j + 1][i] - h00) * v
    h11 = T[j + 1][i + 1]
    return h11 + (T[j + 1][i] - h11) * (1 - u) + (T[j][i + 1] - h11) * (1 - v)


def H(x, y):
    """道路などを載せる高さ（掘り下げ前の地形。メッシュの三角形と同じ割り方で補間）"""
    return _H(TH0, x, y)


def Hs(x, y, r=8.0):
    """周りの最高点（高架の目安。川の窪みに引っぱられないように）"""
    return max(H(x + dx, y + dy) for dx in (-r, 0, r) for dy in (-r, 0, r))


# ---------------------------------------------------------------- OSM 読み込み
osm = json.load(open(OSM_PATH))
NODE, NTAG, WAYS, RELS = {}, {}, {}, []
for e in osm["elements"]:
    if e["type"] == "node":
        NODE[e["id"]] = to_local(e["lat"], e["lon"])
        if "tags" in e:
            NTAG[e["id"]] = e["tags"]
for e in osm["elements"]:
    if e["type"] == "way":
        if e["id"] in WAYS and "tags" not in e:
            continue
        WAYS[e["id"]] = e
    elif e["type"] == "relation":
        RELS.append(e)
for ob in list(bpy.data.objects):
    bpy.data.objects.remove(ob)
log("OSM: 点", len(NODE), "線", len(WAYS), "関係", len(RELS))


def tags(w):
    return w.get("tags", {})


def ring_xy(ids):
    return [NODE[i] for i in ids if i in NODE]


def assemble(rel, role="outer"):
    """マルチポリゴンの構成線をつないで閉じた輪にする"""
    segs = [list(WAYS[m["ref"]]["nodes"]) for m in rel["members"]
            if m["type"] == "way" and m.get("role", "outer") in (role, "") and m["ref"] in WAYS]
    rings = []
    while segs:
        cur = segs.pop(0)
        changed = True
        while cur[0] != cur[-1] and changed:
            changed = False
            for k, s in enumerate(segs):
                if s[0] == cur[-1]:
                    cur += s[1:]
                elif s[-1] == cur[-1]:
                    cur += s[::-1][1:]
                elif s[-1] == cur[0]:
                    cur = s + cur[1:]
                elif s[0] == cur[0]:
                    cur = s[::-1] + cur[1:]
                else:
                    continue
                segs.pop(k)
                changed = True
                break
        if cur[0] == cur[-1] and len(cur) >= 4:
            rings.append(ring_xy(cur))
    return rings


def clip_ring(ring):
    """正方形の範囲で多角形を切り取る（Sutherland–Hodgman）"""
    def clip(pts, inside, inter):
        out = []
        for k in range(len(pts)):
            a, b = pts[k - 1], pts[k]
            ia, ib = inside(a), inside(b)
            if ib:
                if not ia:
                    out.append(inter(a, b))
                out.append(b)
            elif ia:
                out.append(inter(a, b))
        return out

    def lerp_x(a, b, X):
        t = (X - a[0]) / (b[0] - a[0])
        return (X, a[1] + (b[1] - a[1]) * t)

    def lerp_y(a, b, Y):
        t = (Y - a[1]) / (b[1] - a[1])
        return (a[0] + (b[0] - a[0]) * t, Y)

    p = ring[:-1] if ring[0] == ring[-1] else ring[:]
    p = clip(p, lambda q: q[0] >= -HALF, lambda a, b: lerp_x(a, b, -HALF))
    if p:
        p = clip(p, lambda q: q[0] <= HALF, lambda a, b: lerp_x(a, b, HALF))
    if p:
        p = clip(p, lambda q: q[1] >= -HALF, lambda a, b: lerp_y(a, b, -HALF))
    if p:
        p = clip(p, lambda q: q[1] <= HALF, lambda a, b: lerp_y(a, b, HALF))
    return p if len(p) >= 3 else None


def polygons(pred):
    """条件に合う面（閉じた線＋マルチポリゴン）を [外周, 穴...] のリストで返す"""
    out = []
    for w in WAYS.values():
        t = tags(w)
        if pred(t) and len(w["nodes"]) >= 4 and w["nodes"][0] == w["nodes"][-1]:
            r = clip_ring(ring_xy(w["nodes"]))
            if r:
                out.append(([r], t))
    for rel in RELS:
        t = rel.get("tags", {})
        if t.get("type") == "multipolygon" and pred(t):
            outers = [clip_ring(r) for r in assemble(rel, "outer")]
            inners = [clip_ring(r) for r in assemble(rel, "inner")]
            inners = [r for r in inners if r]
            for o in outers:
                if o:
                    out.append(([o] + inners, t))
    return out


def scan_fill(rings, step, origin, n):
    """格子点のうち面の内側にあるもの (i, j) を返す（偶奇規則）"""
    ys = [p[1] for r in rings for p in r]
    j0 = max(0, int(math.ceil((min(ys) - origin) / step)))
    j1 = min(n - 1, int(math.floor((max(ys) - origin) / step)))
    for j in range(j0, j1 + 1):
        y = origin + j * step
        xs = []
        for r in rings:
            for k in range(len(r)):
                (x1, y1), (x2, y2) = r[k - 1], r[k]
                if (y1 > y) != (y2 > y):
                    xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            i0 = max(0, int(math.ceil((a - origin) / step)))
            i1 = min(n - 1, int(math.floor((b - origin) / step)))
            for i in range(i0, i1 + 1):
                yield i, j


def inside(rings, x, y):
    c = False
    for r in rings:
        for k in range(len(r)):
            (x1, y1), (x2, y2) = r[k - 1], r[k]
            if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                c = not c
    return c


# ---------------------------------------------------------------- メッシュ組み立て
class MB:
    def __init__(self):
        self.v, self.f, self.m = [], [], []

    def vert(self, p):
        self.v.append(p)
        return len(self.v) - 1

    def face(self, idx, mat=0):
        self.f.append(idx)
        self.m.append(mat)


MATS = {}


def material(name, rgb, rough=0.8, metal=0.0, vcol=False):
    if name in MATS:
        return MATS[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if vcol:
        node = m.node_tree.nodes.new("ShaderNodeVertexColor")
        node.layer_name = "Col"
        m.node_tree.links.new(node.outputs["Color"], bsdf.inputs["Base Color"])
    m.use_backface_culling = False
    MATS[name] = m
    return m


COLL = {}


def collection(name):
    if name not in COLL:
        c = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(c)
        COLL[name] = c
    return COLL[name]


def make_object(name, coll, mb, mats, smooth=False):
    if not mb.f:
        log("  （空のため省略）", name)
        return None
    me = bpy.data.meshes.new(name)
    me.from_pydata(mb.v, [], mb.f)
    for m in mats:
        me.materials.append(m)
    if len(mats) > 1:
        me.polygons.foreach_set("material_index", mb.m)
    me.validate(clean_customdata=False)
    me.shade_smooth() if smooth else me.shade_flat()
    me.update()
    ob = bpy.data.objects.new(name, me)
    collection(coll).objects.link(ob)
    log(f"  {name}: 頂点 {len(me.vertices):,} / 面 {len(me.polygons):,}")
    return ob


# ---------------------------------------------------------------- 水面と緑地
WATER_PRED = lambda t: (t.get("natural") == "water" or t.get("waterway") == "riverbank"
                        or t.get("landuse") in ("reservoir", "basin"))
GREEN_PRED = lambda t: (t.get("leisure") in ("park", "garden", "pitch", "golf_course", "nature_reserve")
                        or t.get("landuse") in ("grass", "forest", "meadow", "cemetery", "recreation_ground",
                                                "village_green", "allotments", "orchard")
                        or t.get("natural") in ("wood", "scrub", "grassland", "heath"))

water = polygons(WATER_PRED)
green = polygons(GREEN_PRED)
log("水面", len(water), "面 / 緑地", len(green), "面")

water_mb = MB()
WATER_RINGS = []
for rings, t in water:
    river = t.get("water") in ("river", "canal") or t.get("waterway") == "riverbank"
    bnd = sorted(H(x, y) for x, y in rings[0])
    level = 0.8 if river else bnd[len(bnd) // 2] - 0.5  # 川は潮位程度、池は岸の中央値 -0.5m
    for i, j in scan_fill(rings, TSTEP, -HALF, N):
        TH[j][i] = min(TH[j][i], level - 2.0)
    flat = [[(x, y, level) for x, y in r] for r in rings]
    allp = [p for r in flat for p in r]
    base = len(water_mb.v)
    water_mb.v.extend(allp)
    try:
        tris = tessellate_polygon(flat)
    except Exception:
        tris = []
    for tri in tris:
        water_mb.face([base + tri[2], base + tri[1], base + tri[0]])
    WATER_RINGS.append(rings)

# ---------------------------------------------------------------- 地形
log("地形をつくる", N, "x", N)
GREEN_RGB, GROUND_RGB = (0.20, 0.33, 0.14), (0.42, 0.41, 0.38)
gmask = [[False] * N for _ in range(N)]
for rings, t in green:
    for i, j in scan_fill(rings, TSTEP, -HALF, N):
        gmask[j][i] = True
# ---------------------------------------------------------------- 線データの準備
CAR = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential",
       "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
       "living_street", "service", "road", "busway"}
WALK = {"footway": 2.0, "path": 1.5, "pedestrian": 4.0, "steps": 2.0, "cycleway": 2.0,
        "track": 3.0, "bridleway": 1.5, "corridor": 2.0}
RAIL = {"rail", "light_rail", "narrow_gauge", "tram", "monorail", "subway"}
MAJOR = ["motorway", "trunk", "primary", "secondary", "tertiary"]
LANE_W = {"motorway": 3.5, "trunk": 3.25, "primary": 3.25, "secondary": 3.0, "tertiary": 3.0}
SHOULDER = {"motorway": 1.0, "trunk": 0.5, "primary": 0.5, "secondary": 0.4, "tertiary": 0.4}
UNMARKED_W = {"unclassified": 5.0, "residential": 4.0, "living_street": 3.5, "service": 3.0, "road": 4.0,
              "busway": 3.5}
RANK = {"motorway": 9, "trunk": 8, "primary": 7, "secondary": 6, "tertiary": 5, "unclassified": 3,
        "residential": 2, "living_street": 2, "road": 2, "busway": 2, "service": 1}


def num(s, default=None):
    if s is None:
        return default
    try:
        return float(str(s).split(";")[0].replace("m", "").strip())
    except ValueError:
        return default


def is_bridge(t):
    return t.get("bridge") not in (None, "no")


def is_tunnel(t):
    return t.get("tunnel") not in (None, "no", "building_passage") or t.get("covered") == "yes" and t.get("layer", "0").startswith("-")


def base_class(hw):
    return hw.replace("_link", "")


def profile(t):
    """車線数・幅などの断面。日本の標準的な値で補う"""
    hw = t["highway"]
    bc = base_class(hw)
    link = hw.endswith("_link")
    oneway = t.get("oneway") in ("yes", "true", "1", "-1") or hw in ("motorway", "motorway_link") \
        or t.get("junction") == "roundabout"
    lanes = num(t.get("lanes"))
    lanes = int(lanes) if lanes else None
    marked = bc in LANE_W or (lanes is not None and lanes >= 2)
    p = {"hw": hw, "oneway": oneway, "marked": marked, "lanes_tagged": lanes is not None}
    if marked:
        if lanes is None:
            if link:
                lanes = 1
            elif oneway:
                lanes = 2 if bc in ("motorway", "trunk", "primary") else 1
            else:
                lanes = 4 if bc == "trunk" else 2
        lanes = max(1, min(lanes, 8))
        lw = LANE_W.get(bc, 2.75)
        sh = SHOULDER.get(bc, 0.25) * (0.75 if link else 1)
        width = num(t.get("width"))
        if width and 2.5 < width < 40:
            lw = max(2.5, min(4.0, (width - 2 * sh) / lanes))
        if oneway:
            lf, lb = lanes, 0
        else:
            lf = int(num(t.get("lanes:forward"), 0) or 0)
            lb = int(num(t.get("lanes:backward"), 0) or 0)
            if lf + lb != lanes or lf == 0 or lb == 0:
                lf = max(1, (lanes + 1) // 2)
                lb = max(1, lanes - lf)
        W = (lf + lb) * lw + 2 * sh
        p.update(lanes=lf + lb, lf=lf, lb=lb, lw=lw, sh=sh, W=W)
    else:
        W = UNMARKED_W.get(hw, 4.0)
        if hw == "service" and t.get("service") in ("driveway", "drive-through"):
            W = 2.75
        if hw == "service" and t.get("service") == "parking_aisle":
            W = 5.0
        width = num(t.get("width"))
        if width and 2 < width < 15:
            W = width
        p.update(lanes=1, lf=1, lb=0 if oneway else 1, lw=W, sh=0, W=W)
    # 歩道（車道の外側に段差 15cm で付ける）
    sw = t.get("sidewalk", t.get("sidewalk:both"))
    side = None
    if sw in ("both", "left", "right"):
        side = sw
    elif sw in ("no", "none", "separate"):
        side = None
    elif bc in ("trunk", "primary", "secondary", "tertiary") and not link:
        side = "left" if oneway else "both"  # 上下分離の片側は外側（進行方向の左）だけ
    p["sidewalk"] = side
    p["sw_w"] = 3.0 if bc in ("trunk", "primary") else 2.5
    p["center"] = "yellow" if (not oneway and p["lanes"] >= 4) else "white"
    p["dash"] = (8.0, 12.0) if bc == "motorway" else (5.0, 5.0)
    p["line_w"] = 0.20 if bc == "motorway" else 0.15
    return p


def clip_polyline(ids):
    """正方形の内側だけを取り出す。[(x, y, node_id|None), ...] の列を返す"""
    pts = [(NODE[i][0], NODE[i][1], i) for i in ids if i in NODE]
    pieces, cur = [], []

    def ins(p):
        return -HALF <= p[0] <= HALF and -HALF <= p[1] <= HALF

    for k in range(len(pts)):
        p = pts[k]
        if k == 0:
            if ins(p):
                cur.append(p)
            continue
        a = pts[k - 1]
        # 線分 a→p を正方形で切る（Liang–Barsky）
        t0, t1 = 0.0, 1.0
        dx, dy = p[0] - a[0], p[1] - a[1]
        ok = True
        for pp, qq in ((-dx, a[0] + HALF), (dx, HALF - a[0]), (-dy, a[1] + HALF), (dy, HALF - a[1])):
            if pp == 0:
                if qq < 0:
                    ok = False
            else:
                r = qq / pp
                if pp < 0:
                    t0 = max(t0, r)
                else:
                    t1 = min(t1, r)
        if not ok or t0 > t1:
            if len(cur) >= 2:
                pieces.append(cur)
            cur = []
            continue
        if t0 > 0:
            if len(cur) >= 2:
                pieces.append(cur)
            cur = [(a[0] + dx * t0, a[1] + dy * t0, None)]
        if t1 < 1:
            cur.append((a[0] + dx * t1, a[1] + dy * t1, None))
            if len(cur) >= 2:
                pieces.append(cur)
            cur = []
        else:
            cur.append(p)
    if len(cur) >= 2:
        pieces.append(cur)
    return pieces


class Line:
    """1本の線（道路・線路・歩行者道）。中心線を 4m 以下に細かく刻んで持つ"""
    SEG = 4.0

    def __init__(self, way, kind, pts):
        self.way, self.kind, self.t = way, kind, tags(way)
        self.bridge = is_bridge(self.t)
        s = [0.0]
        for k in range(1, len(pts)):
            s.append(s[-1] + math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]))
        self.length = s[-1]
        self.nodes = [(s[k], pts[k][2]) for k in range(len(pts))]
        smp = []
        for k in range(len(pts) - 1):
            (x1, y1, _), (x2, y2, _) = pts[k], pts[k + 1]
            L = s[k + 1] - s[k]
            if L < 1e-6:
                continue
            tx, ty = (x2 - x1) / L, (y2 - y1) / L
            n = max(1, int(math.ceil(L / self.SEG)))
            for q in range(n):
                f = q / n
                smp.append([s[k] + L * f, x1 + (x2 - x1) * f, y1 + (y2 - y1) * f, -ty, tx, 1.0])
        if not smp:
            self.smp = []
            return
        last = pts[-1]
        smp.append([self.length, last[0], last[1], smp[-1][3], smp[-1][4], 1.0])
        # 曲がり角は左右の法線を平均（留め継ぎ）
        node_s = set(s[1:-1])
        for k in range(1, len(smp) - 1):
            if smp[k][0] in node_s:
                a, b = smp[k - 1], smp[k + 1]
                n1 = (-(smp[k][2] - a[2]), smp[k][1] - a[1])
                n2 = (-(b[2] - smp[k][2]), b[1] - smp[k][1])
                l1, l2 = math.hypot(*n1) or 1, math.hypot(*n2) or 1
                n1, n2 = (n1[0] / l1, n1[1] / l1), (n2[0] / l2, n2[1] / l2)
                bx, by = n1[0] + n2[0], n1[1] + n2[1]
                bl = math.hypot(bx, by)
                if bl > 1e-6:
                    bx, by = bx / bl, by / bl
                    smp[k][3], smp[k][4] = bx, by
                    smp[k][5] = 1.0 / max(0.5, bx * n1[0] + by * n1[1])
        self.smp = smp
        self.S = [p[0] for p in smp]
        self.deck = None  # 高架なら各サンプルの床面高さ

    def at(self, s):
        """s の位置の [x, y, nx, ny, miter, deck]"""
        s = min(max(s, 0.0), self.length)
        k = bisect_left(self.S, s)
        if k <= 0:
            k = 1
        if k >= len(self.S):
            k = len(self.S) - 1
        a, b = self.smp[k - 1], self.smp[k]
        f = 0 if b[0] == a[0] else (s - a[0]) / (b[0] - a[0])
        nx, ny = a[3] + (b[3] - a[3]) * f, a[4] + (b[4] - a[4]) * f
        nl = math.hypot(nx, ny) or 1
        d = None
        if self.deck is not None:
            d = self.deck[k - 1] + (self.deck[k] - self.deck[k - 1]) * f
        return [a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f, nx / nl, ny / nl,
                a[5] + (b[5] - a[5]) * f, d]

    def stations(self, s0, s1):
        s0, s1 = max(0.0, s0), min(self.length, s1)
        if s1 - s0 < 0.05:
            return []
        out = [self.at(s0)]
        k = bisect_left(self.S, s0 + 1e-6)
        while k < len(self.S) and self.S[k] < s1 - 1e-6:
            p = self.smp[k]
            out.append([p[1], p[2], p[3], p[4], p[5], self.deck[k] if self.deck is not None else None])
            k += 1
        out.append(self.at(s1))
        return out

    def z(self, st, x, y, off):
        return (st[5] if st[5] is not None else H(x, y)) + off

    COL = 4.0  # 横方向にも 4m 以下に刻んで坂や尾根で地面が突き抜けないようにする

    def columns(self, o1, o2):
        n = max(1, int(math.ceil((o2 - o1) / self.COL - 1e-9)))
        return [o1 + (o2 - o1) * q / n for q in range(n + 1)]

    def surface(self, st, o):
        """車道の表面の高さ（車道の帯と同じ刻みで直線補間。白線はこの上に載せる）"""
        if st[5] is not None:
            return st[5]
        cols = self.road_cols
        o = min(max(o, cols[0]), cols[-1])
        k = min(max(bisect_left(cols, o), 1), len(cols) - 1)
        a, b = cols[k - 1], cols[k]
        x, y, nx, ny, m = st[:5]
        ha = H(x + nx * m * a, y + ny * m * a)
        hb = H(x + nx * m * b, y + ny * m * b)
        return ha + (hb - ha) * ((o - a) / (b - a) if b > a else 0) + Z_ROAD

    def strip(self, mb, s0, s1, o1, o2, off, mat=0, on_road=False):
        """中心から左へ o1〜o2 [m] の帯を s0〜s1 区間に貼る
        on_road=True なら車道の表面から off の高さ（白線など）、そうでなければ地面から off"""
        cols = [o1, o2] if on_road else self.columns(o1, o2)
        prev = None
        for st in self.stations(s0, s1):
            x, y, nx, ny, m = st[:5]
            cur = []
            for o in cols:
                px, py = x + nx * m * o, y + ny * m * o
                z = self.surface(st, o) + off if on_road else self.z(st, px, py, off)
                cur.append(mb.vert((px, py, z)))
            if prev:
                for q in range(len(cols) - 1):
                    mb.face([prev[q], cur[q], cur[q + 1], prev[q + 1]], mat)
            prev = cur

    def wall(self, mb, s0, s1, o, off_lo, off_hi, mat=0):
        """o の位置に縦の面（縁石・桁の側面）"""
        prev = None
        for st in self.stations(s0, s1):
            x, y, nx, ny, m = st[:5]
            px, py = x + nx * m * o, y + ny * m * o
            cur = (mb.vert((px, py, self.z(st, px, py, off_lo))), mb.vert((px, py, self.z(st, px, py, off_hi))))
            if prev:
                mb.face([prev[0], cur[0], cur[1], prev[1]], mat)
            prev = cur


# 道路・線路・歩行者道を集める
lines = []
for w in WAYS.values():
    t = tags(w)
    if "tags" not in w or t.get("area") == "yes":
        continue
    kind = None
    if t.get("highway") in CAR:
        kind = "car"
    elif t.get("highway") in WALK and t.get("footway") != "crossing" and "indoor" not in t:
        kind = "walk"
    elif t.get("railway") in RAIL:
        kind = "rail"
    if not kind:
        continue
    lvl = num(t.get("level"), 0) or 0
    if lvl < 0:
        continue
    ids = list(w["nodes"])
    if kind == "car" and t.get("oneway") == "-1":
        ids.reverse()
    w = dict(w, nodes=ids)
    tun = is_tunnel(t) or t.get("railway") == "subway" and not is_bridge(t)
    for piece in clip_polyline(ids):
        L = Line(w, kind, piece)
        if L.smp:
            L.tunnel = tun
            L.prof = profile(t) if kind == "car" else None
            lines.append(L)
log("線:", sum(1 for L in lines if L.kind == "car"), "道路 /", sum(1 for L in lines if L.kind == "walk"),
    "歩行者道 /", sum(1 for L in lines if L.kind == "rail"), "線路（トンネル含む）")

# ---------------------------------------------------------------- 高架の高さ
# 地上の線とつながる点 = 「地面の高さに固定」。そこから坂の上限を守りながら目標の高さまで上げる
by_node = {}
for L in lines:
    for s, nid in L.nodes:
        if nid is not None:
            by_node.setdefault(nid, []).append((L, s))

walk_bridge_nodes = {nid for L in lines if L.kind == "walk" and L.bridge for _, nid in L.nodes if nid}
for L in lines:  # 歩道橋の階段は高架の一部として扱う（上端が橋の高さになる）
    if L.kind == "walk" and not L.bridge and L.t.get("highway") == "steps":
        if any(nid in walk_bridge_nodes for _, nid in L.nodes if nid):
            L.bridge = True
            L.ramp = True

elev = [L for L in lines if L.bridge and not L.tunnel]
anchor = {}
for L in lines:
    if not L.bridge or L.tunnel:
        for s, nid in L.nodes:
            if nid is not None and nid in NODE:
                anchor[nid] = H(*NODE[nid])
adj, target, grade = {}, {}, {}


def clearance(L):
    lay = max(1.0, num(L.t.get("layer"), 1) or 1)
    if L.kind == "rail":
        return 8.0 * lay, 0.035
    if L.kind == "walk":
        return 5.5, 0.5
    if L.prof["hw"].startswith("motorway"):
        return max(10.0, 6.5 * lay), 0.06
    return 6.5 * lay, 0.06


for L in elev:
    clr, g = clearance(L)
    ns = [(s, nid) for s, nid in L.nodes]
    for k, (s, nid) in enumerate(ns):
        key = nid if nid is not None else ("cut", id(L), k)
        x, y = L.at(s)[:2]
        target[key] = max(target.get(key, -1e9), Hs(x, y) + clr)
        grade[key] = max(grade.get(key, 0), g)
        adj.setdefault(key, [])
        if k:
            pk = ns[k - 1][1] if ns[k - 1][1] is not None else ("cut", id(L), k - 1)
            d = s - ns[k - 1][0]
            if d > 1e-6:
                adj[key].append((pk, d))
                adj[pk].append((key, d))

# つながった高架のまとまりごとに処理
comp, seen = [], set()
for v in adj:
    if v in seen:
        continue
    stack, cc = [v], []
    seen.add(v)
    while stack:
        u = stack.pop()
        cc.append(u)
        for w2, _ in adj[u]:
            if w2 not in seen:
                seen.add(w2)
                stack.append(w2)
    comp.append(cc)

deck_h = {}
for cc in comp:
    anchors = [v for v in cc if v in anchor]
    length = sum(d for v in cc for _, d in adj[v]) / 2
    # 川の上の短い橋は、両岸を直線で結ぶだけ（持ち上げない）
    over_water = False
    for v in cc:
        if not isinstance(v, tuple) and v in NODE:
            x, y = NODE[v]
            if any(inside(r, x, y) for r in WATER_RINGS):
                over_water = True
                break
    lift_ok = not (over_water and length < 150)
    base = {v: (anchor[v] if v in anchor else target[v] - 0.0) for v in cc}
    if anchors:
        # 端点を固定して、間を距離に比例してつなぐ（ラプラス方程式を反復で解く）
        free = [v for v in cc if v not in anchor]
        for v in free:
            base[v] = sum(anchor[a] for a in anchors) / len(anchors)
        for it in range(20000):
            delta = 0.0
            for v in free:
                sw = sum(1 / d for _, d in adj[v])
                nv = sum(base[u] / d for u, d in adj[v]) / sw if sw else base[v]
                delta = max(delta, abs(nv - base[v]))
                base[v] = nv
            if delta < 1e-4:
                break
        # 一番近い固定点までの距離
        dist = {v: 0.0 for v in anchors}
        pq = [(0.0, v) for v in anchors]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e18):
                continue
            for w2, l in adj[u]:
                if d + l < dist.get(w2, 1e18):
                    dist[w2] = d + l
                    heapq.heappush(pq, (d + l, w2))
        for v in cc:
            lift = 0.0
            if lift_ok and v not in anchor:
                lift = max(0.0, min(target[v] - base[v], grade[v] * dist.get(v, 0)))
            deck_h[v] = base[v] + lift
    else:
        for v in cc:
            deck_h[v] = target[v]
log("高架:", len(comp), "まとまり /", len(deck_h), "点")

for L in elev:
    ds = []
    for k, (s, nid) in enumerate(L.nodes):
        key = nid if nid is not None else ("cut", id(L), k)
        ds.append((s, deck_h[key]))
    S = [d[0] for d in ds]
    L.deck = []
    for p in L.smp:
        k = min(max(bisect_left(S, p[0]), 1), len(S) - 1)
        (s0, h0), (s1, h1) = ds[k - 1], ds[k]
        f = 0 if s1 == s0 else (p[0] - s0) / (s1 - s0)
        L.deck.append(h0 + (h1 - h0) * f)

# ---------------------------------------------------------------- 交差点・横断歩道
cars = [L for L in lines if L.kind == "car" and not L.tunnel]
arms = {}
for L in cars:
    if L.prof["hw"] == "service":
        continue
    n = len(L.nodes)
    for k, (s, nid) in enumerate(L.nodes):
        if nid is None:
            continue
        arms.setdefault(nid, []).append((L, (k > 0) + (k < n - 1)))
junction = {nid for nid, lst in arms.items() if sum(a for _, a in lst) >= 3}
signal_nodes = {nid for nid, t in NTAG.items() if t.get("highway") == "traffic_signals"}
log("交差点", len(junction), "/ 信号", len(signal_nodes))

road_mb, side_mb = MB(), MB()
white_mb, yellow_mb, zebra_mb, stop_mb = MB(), MB(), MB(), MB()
walk_mb, railbed_mb, rails_mb, struct_mb = MB(), MB(), MB(), MB()
Z_ROAD, Z_MARK, Z_SIDE, Z_WALK = 0.15, 0.20, 0.30, 0.22
lanes_out = []


def subtract(intervals, cuts):
    """区間 [a,b] の列から cuts を取り除く"""
    out = intervals
    for c0, c1 in cuts:
        nxt = []
        for a, b in out:
            if c1 <= a or c0 >= b:
                nxt.append((a, b))
                continue
            if c0 > a:
                nxt.append((a, c0))
            if c1 < b:
                nxt.append((c1, b))
        out = nxt
    return [(a, b) for a, b in out if b - a > 0.3]


def dashes(intervals, on, off):
    out = []
    for a, b in intervals:
        s = a + 1.0
        while s < b:
            out.append((s, min(s + on, b)))
            s += on + off
    return out


for L in cars:
    p = L.prof
    W = p["W"]
    L.road_cols = L.columns(-W / 2, W / 2)
    L.strip(road_mb, 0, L.length, -W / 2, W / 2, Z_ROAD if L.deck is None else 0.0)
    # 交差点で止める区間
    mark_cut, side_cut, cross = [], [], []
    for s, nid in L.nodes:
        if nid is None:
            continue
        if nid in junction:
            others = [M for M, _ in arms.get(nid, []) if M is not L]
            if others:
                rm = max(M.prof["W"] / 2 for M in others) + 2.0
                minor = all(RANK[M.prof["hw"].replace("_link", "")] < RANK[base_class(p["hw"])] and
                            not M.prof["marked"] for M in others)
                if not (minor and p["marked"]):  # 優先道路の線は細い道との交差で途切れない
                    mark_cut.append((s - rm, s + rm))
                rs = max(M.prof["W"] / 2 for M in others) + 0.5
                side_cut.append((s - rs, s + rs))
        nt = NTAG.get(nid, {})
        if nt.get("highway") == "crossing" and nt.get("crossing") not in ("no", "unmarked", "informal"):
            cross.append((s, nid))
            mark_cut.append((s - 2.5, s + 2.5))
            side_cut.append((s - 2.0, s + 2.0))
    full = [(0.0, L.length)]
    zoff = Z_MARK - Z_ROAD  # 車道の表面から 5cm 上
    # 歩道（段差つき）
    if p["sidewalk"] and L.deck is None:
        segs = subtract(full, side_cut)
        sides = {"both": (1, -1), "left": (1,), "right": (-1,)}[p["sidewalk"]]
        for sg in sides:
            for a, b in segs:
                o1, o2 = W / 2, W / 2 + p["sw_w"]
                if sg < 0:
                    o1, o2 = -o2, -o1
                L.strip(side_mb, a, b, o1, o2, Z_SIDE)
                edge = W / 2 * sg
                L.wall(side_mb, a, b, edge, Z_ROAD, Z_SIDE)
    # 区画線
    lw, sh, lf, lb, lwid = p["lw"], p["sh"], p["lf"], p["lb"], p["line_w"]
    left = W / 2 - sh  # 進行方向の左端（日本は左側通行）
    segs = subtract(full, mark_cut)
    if p["marked"]:
        for a, b in segs:
            L.strip(white_mb, a, b, left - lwid, left, zoff, on_road=True)  # 外側線
            L.strip(white_mb, a, b, -left, -left + lwid, zoff, on_road=True)
        for k in range(1, lf):  # 同じ向きの車線境界（破線）
            o = left - k * lw
            for a, b in dashes(segs, *p["dash"]):
                L.strip(white_mb, a, b, o - lwid / 2, o + lwid / 2, zoff, on_road=True)
        for k in range(1, lb):
            o = left - lf * lw - k * lw
            for a, b in dashes(segs, *p["dash"]):
                L.strip(white_mb, a, b, o - lwid / 2, o + lwid / 2, zoff, on_road=True)
        if lb > 0:  # 中央線
            o = left - lf * lw
            mb = yellow_mb if p["center"] == "yellow" else white_mb
            for a, b in segs:
                L.strip(mb, a, b, o - lwid / 2, o + lwid / 2, zoff, on_road=True)
    # 横断歩道（縞は道路と平行）と、信号のある所の停止線
    for s, nid in cross:
        o = -W / 2 + 0.5
        while o + 0.45 <= W / 2 - 0.5:
            L.strip(zebra_mb, s - 2.0, s + 2.0, o, o + 0.45, zoff, on_road=True)
            o += 0.9
        if p["marked"]:
            near_sig = nid in signal_nodes or NTAG.get(nid, {}).get("crossing") == "traffic_signals" or \
                any(abs(s2 - s) < 30 and n2 in signal_nodes for s2, n2 in L.nodes)
            if near_sig:
                d = 2.0 + 2.0 + 0.225
                L.strip(stop_mb, s - d - 0.225, s - d + 0.225, left - lf * lw, left, zoff, on_road=True)  # 手前から来る車
                if lb > 0:
                    L.strip(stop_mb, s + d - 0.225, s + d + 0.225, -left, left - lf * lw, zoff, on_road=True)
    # 車線の中心線（アプリで車を走らせる用）
    sts = L.stations(0, L.length)

    def lane_path(o, reverse):
        pts = []
        for st in sts:
            x, y, nx, ny, m = st[:5]
            px, py = x + nx * m * o, y + ny * m * o
            pts.append([round(px, 2), round(L.surface(st, o), 2), round(-py, 2)])
        return pts[::-1] if reverse else pts

    ln = []
    if p["marked"]:
        for k in range(lf):
            ln.append({"dir": "forward", "index": k, "path": lane_path(left - (k + 0.5) * lw, False)})
        for k in range(lb):
            ln.append({"dir": "backward", "index": k, "path": lane_path(-left + (k + 0.5) * lw, True)})
    else:
        ln.append({"dir": "both" if not p["oneway"] else "forward", "index": 0, "path": lane_path(0.0, False)})
    lanes_out.append({
        "way": L.way["id"], "highway": p["hw"], "name": L.t.get("name"), "oneway": p["oneway"],
        "lanes": p["lanes"], "lanes_from_osm": p["lanes_tagged"], "width": round(W, 2),
        "bridge": L.deck is not None, "from_node": L.nodes[0][1], "to_node": L.nodes[-1][1],
        "maxspeed": L.t.get("maxspeed"), "lane_paths": ln})

# 歩行者道
for L in lines:
    if L.kind != "walk" or L.tunnel:
        continue
    w = num(L.t.get("width"), WALK.get(L.t.get("highway"), 2.0)) or 2.0
    w = max(1.0, min(w, 8.0))
    L.strip(walk_mb, 0, L.length, -w / 2, w / 2, Z_WALK if L.deck is None else 0.0)

# 線路（道床＋レール2本）
for L in lines:
    if L.kind != "rail" or L.tunnel:
        continue
    gauge = (num(L.t.get("gauge"), 1067) or 1067) / 1000.0
    base = 0.3 if L.deck is None else 0.0
    L.strip(railbed_mb, 0, L.length, -1.7, 1.7, base)
    for sg in (1, -1):
        c = sg * gauge / 2
        L.strip(rails_mb, 0, L.length, c - 0.04, c + 0.04, base + 0.17)

# 高架の構造（桁の側面・底、橋脚）
PIER_EVERY = 30.0
for L in elev:
    if L.kind == "car":
        half = L.prof["W"] / 2 + (L.prof["sw_w"] if L.prof["sidewalk"] else 0)
        pier = 1.8
    elif L.kind == "rail":
        half, pier = 1.9, 1.4
    else:
        w = num(L.t.get("width"), WALK.get(L.t.get("highway"), 2.0)) or 2.0
        half, pier = w / 2, 0.5
    if getattr(L, "ramp", False):
        continue
    girder = 1.6 if L.kind != "walk" else 0.6
    for sg in (1, -1):
        L.wall(struct_mb, 0, L.length, sg * half, -girder, 0.0)
    L.strip(struct_mb, 0, L.length, -half, half, -girder)
    s = PIER_EVERY / 2
    while s < L.length:
        x, y, nx, ny, m, d = L.at(s)
        g = H(x, y)
        if d is not None and d - girder - g > 2.5:
            tx, ty = ny, -nx
            r = pier / 2
            corners = [(x + tx * r * a + nx * r * b, y + ty * r * a + ny * r * b) for a, b in
                       ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            lo = [struct_mb.vert((cx, cy, g - 0.5)) for cx, cy in corners]
            hi = [struct_mb.vert((cx, cy, d - girder)) for cx, cy in corners]
            for q in range(4):
                struct_mb.face([lo[q], lo[(q + 1) % 4], hi[(q + 1) % 4], hi[q]])
        s += PIER_EVERY

# ---------------------------------------------------------------- 建物（PLATEAU）
log("建物を読み込む")
wall_m = material("building_wall", (0.62, 0.62, 0.64), 0.85)
roof_m = material("building_roof", (0.36, 0.37, 0.40), 0.9)
bmbs, bindex = {}, []
for line in open(BLDG_PATH):
    b = json.loads(line)
    mb = bmbs.setdefault(b["mesh"], MB())
    if not hasattr(mb, "bid"):
        mb.bid = []
    bi = len(bindex)
    bindex.append({"i": bi, "id": b["id"], "height": b["h"], "lod": b["lod"], "mesh": b["mesh"]})
    zmin = min(p[2] for f in b["faces"] for r in f[1:] for p in r)
    vmap = {}

    def V(p):
        z = p[2] if p[2] > zmin + 0.05 else zmin - 1.5  # 足元を 1.5m 埋めて浮きを防ぐ
        key = (p[0], p[1], z)
        if key not in vmap:
            vmap[key] = mb.vert(key)
            mb.bid.append(bi)
        return vmap[key]

    for f in b["faces"]:
        kind, rings = f[0], f[1:]
        outer = rings[0]
        if kind == "?":  # LOD1：面の向きで屋根・壁・底を判定
            nx = ny = nz = 0.0
            for k in range(len(outer)):
                a, c = outer[k - 1], outer[k]
                nx += (a[1] - c[1]) * (a[2] + c[2])
                ny += (a[2] - c[2]) * (a[0] + c[0])
                nz += (a[0] - c[0]) * (a[1] + c[1])
            ln_ = math.sqrt(nx * nx + ny * ny + nz * nz) or 1
            nz /= ln_
            kind = "R" if nz > 0.5 else ("G" if nz < -0.5 else "W")
        if kind == "G":
            continue
        mat = 1 if kind == "R" else 0
        if len(rings) == 1:
            idx = []
            for p in outer:
                v = V(p)
                if not idx or idx[-1] != v:
                    idx.append(v)
            if len(idx) > 1 and idx[0] == idx[-1]:
                idx.pop()
            if len(set(idx)) == len(idx) >= 3:
                mb.face(idx, mat)
        else:
            allp = [p for r in rings for p in r]
            ids = [V(p) for p in allp]
            for tri in tessellate_polygon([[tuple(p) for p in r] for r in rings]):
                t3 = [ids[tri[0]], ids[tri[1]], ids[tri[2]]]
                if len(set(t3)) == 3:
                    mb.face(t3, mat)
log("建物", len(bindex), "棟")

# ---------------------------------------------------------------- 地形を道路の下へ逃がす
# 地形は 10m 間隔なので、坂や掘割では道路の帯より上に出ることがある。
# 道路・歩道・線路の頂点の真下で地形が近すぎたら、その三角形の角を道路より 15cm 下げる
def probe_points(mb):
    """頂点に加えて、面の中心と辺の中点も調べる（地形の尾根が帯の途中に入る場合）"""
    yield from mb.v
    V = mb.v
    for f in mb.f:
        ps = [V[k] for k in f]
        n = len(ps)
        yield tuple(sum(p[c] for p in ps) / n for c in range(3))
        for k in range(n):
            a, b = ps[k - 1], ps[k]
            yield ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2)


def clear_under(mb):
    n = 0
    for x, y, z in probe_points(mb):
        if _H(TH, x, y) <= z - 0.08:
            continue
        fx = min(max((x + HALF) / TSTEP, 0.0), N - 1 - 1e-9)
        fy = min(max((y + HALF) / TSTEP, 0.0), N - 1 - 1e-9)
        i, j = int(fx), int(fy)
        corners = [(i, j), (i + 1, j), (i, j + 1)] if (fx - i) + (fy - j) <= 1 else \
            [(i + 1, j + 1), (i, j + 1), (i + 1, j)]
        for ci, cj in corners:
            if TH[cj][ci] > z - 0.15:
                TH[cj][ci] = z - 0.15
                n += 1
    return n


for _ in range(2):
    moved = sum(clear_under(mb) for mb in (road_mb, side_mb, walk_mb, railbed_mb))
    log("地形を道路の下へ下げた頂点", moved)

# ---------------------------------------------------------------- 地形メッシュ
ter = MB()
for j in range(N):
    for i in range(N):
        ter.v.append((-HALF + i * TSTEP, -HALF + j * TSTEP, TH[j][i]))
for j in range(N - 1):
    for i in range(N - 1):
        a, b, c, d = j * N + i, j * N + i + 1, (j + 1) * N + i, (j + 1) * N + i + 1
        ter.face([a, b, c])
        ter.face([b, d, c])
skirt = MB()
FLOOR = -20.0
for side in range(4):
    idx = []
    for k in range(N):
        i, j = [(k, 0), (N - 1, k), (N - 1 - k, N - 1), (0, N - 1 - k)][side]
        idx.append(((-HALF + i * TSTEP, -HALF + j * TSTEP), TH[j][i]))
    for k in range(N - 1):
        (p, h1), (q, h2) = idx[k], idx[k + 1]
        a = skirt.vert((p[0], p[1], h1))
        b = skirt.vert((q[0], q[1], h2))
        c = skirt.vert((q[0], q[1], FLOOR))
        d = skirt.vert((p[0], p[1], FLOOR))
        skirt.face([a, d, c, b])


# ---------------------------------------------------------------- オブジェクト化
log("オブジェクトを作る")
m_ground = material("terrain_ground", (1, 1, 1), 0.95, vcol=True)
make_object("terrain", "01_terrain", ter, [m_ground], smooth=True)
tob = bpy.data.objects["terrain"]
col = tob.data.color_attributes.new("Col", "BYTE_COLOR", "POINT")
cols = []
for j in range(N):
    for i in range(N):
        cols.extend((*(GREEN_RGB if gmask[j][i] else GROUND_RGB), 1.0))
col.data.foreach_set("color", cols)
make_object("terrain_skirt", "01_terrain", skirt, [material("terrain_skirt", (0.25, 0.23, 0.21), 0.95)])
make_object("water", "02_water", water_mb, [material("water", (0.04, 0.12, 0.20), 0.15)])
make_object("road_surface", "03_roads", road_mb, [material("asphalt", (0.055, 0.057, 0.062), 0.9)])
make_object("road_sidewalk", "03_roads", side_mb, [material("sidewalk", (0.33, 0.31, 0.29), 0.9)])
make_object("road_footpath", "03_roads", walk_mb, [material("footpath", (0.40, 0.37, 0.33), 0.9)])
white = material("marking_white", (0.80, 0.80, 0.78), 0.6)
make_object("marking_lane_white", "04_markings", white_mb, [white])
make_object("marking_center_yellow", "04_markings", yellow_mb, [material("marking_yellow", (0.85, 0.50, 0.03), 0.6)])
make_object("marking_crosswalk", "04_markings", zebra_mb, [white])
make_object("marking_stopline", "04_markings", stop_mb, [white])
make_object("rail_bed", "05_rail", railbed_mb, [material("rail_bed", (0.13, 0.11, 0.10), 0.95)])
make_object("rail_track", "05_rail", rails_mb, [material("rail_steel", (0.45, 0.45, 0.47), 0.35, 0.8)])
make_object("structure_viaduct", "06_structures", struct_mb, [material("concrete", (0.40, 0.40, 0.38), 0.9)])
for mesh in sorted(bmbs):
    mb = bmbs[mesh]
    ob = make_object(f"building_{mesh}", "07_buildings", mb, [wall_m, roof_m])
    if ob:  # 頂点ごとに「何番目の建物か」を持たせる（three.js で _BLDG 属性として読める）
        at = ob.data.attributes.new("_BLDG", "FLOAT", "POINT")
        at.data.foreach_set("value", [float(x) for x in mb.bid[:len(ob.data.vertices)]])

# ---------------------------------------------------------------- 保存と書き出し
bpy.context.scene.unit_settings.system = "METRIC"
for ob in list(bpy.data.objects):
    if ob.type != "MESH":
        bpy.data.objects.remove(ob)
bpy.context.preferences.filepaths.save_version = 0  # .blend1 の控えを作らない
blend = os.path.join(OUT, "bandobashi_3dmap.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend)
log("保存:", blend)


def export(path, objs):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    kw = dict(filepath=path, export_format="GLB", use_selection=True, export_apply=True,
              export_yup=True, export_attributes=True)
    try:
        bpy.ops.export_scene.gltf(**kw, export_vertex_color="ACTIVE")
    except TypeError:
        bpy.ops.export_scene.gltf(**kw)
    log(f"書き出し: {os.path.basename(path)}  {os.path.getsize(path) / 1e6:.1f} MB")


glb = os.path.join(OUT, "glb")
os.makedirs(glb, exist_ok=True)
for cname, fname in [("01_terrain", "terrain"), ("02_water", "water"), ("03_roads", "roads"),
                     ("04_markings", "markings"), ("05_rail", "rail"), ("06_structures", "structures"),
                     ("07_buildings", "buildings")]:
    if cname in COLL and COLL[cname].objects:
        export(os.path.join(glb, f"{fname}.glb"), list(COLL[cname].objects))
export(os.path.join(glb, "bandobashi_all.glb"), [o for o in bpy.data.objects])

json.dump({"origin": {"lat": LAT0, "lon": LON0, "note": "glTF: X=東, Y=標高(TP m), Z=南"},
           "ways": lanes_out}, open(os.path.join(OUT, "lanes.json"), "w"), ensure_ascii=False)
json.dump(bindex, open(os.path.join(OUT, "buildings_index.json"), "w"), ensure_ascii=False)
open(os.path.join(OUT, "build_log.txt"), "w").write("\n".join(LOG) + "\n")
log("完了")
