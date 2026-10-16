"""PLATEAU 建物 CityGML → 局所座標の面リスト（JSON Lines）

使い方: python3 -I prep_plateau.py <GMLフォルダ> <出力.jsonl>
- 屋根形状(LOD2)がある建物は LOD2 の屋根・壁を使い、無い建物は箱形(LOD1)を使う
- 建物の足元の中心が 3km 四方の外にあるものは捨てる
1行 = 1棟: {"id", "mesh", "h"(実測高さ), "lod", "faces": [[種類, [外周], [穴]...], ...]}
  種類: R=屋根 W=壁 G=底 / 座標は [x東, y北, z標高] (m, 小数2桁)
"""
import os
import sys
import json
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import to_local, HALF  # noqa: E402

NS = {
    "gml": "http://www.opengis.net/gml",
    "bldg": "http://www.opengis.net/citygml/building/2.0",
}
G = "{%s}" % NS["gml"]
B = "{%s}" % NS["bldg"]
KIND = {B + "RoofSurface": "R", B + "WallSurface": "W", B + "GroundSurface": "G"}


def ring(lr):
    vals = lr.find(G + "posList").text.split()
    pts = []
    for i in range(0, len(vals) - 2, 3):
        lat, lon, h = float(vals[i]), float(vals[i + 1]), float(vals[i + 2])
        x, y = to_local(lat, lon)
        pts.append([round(x, 2), round(y, 2), round(h, 2)])
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    return pts


def polys(elem):
    out = []
    for p in elem.iter(G + "Polygon"):
        ext = p.find(G + "exterior/" + G + "LinearRing")
        if ext is None:
            continue
        rings = [ring(ext)]
        for it in p.findall(G + "interior/" + G + "LinearRing"):
            rings.append(ring(it))
        if len(rings[0]) >= 3:
            out.append(rings)
    return out


def building(el, mesh):
    faces = []
    lod = 1
    for bb in el.findall(B + "boundedBy"):
        for surf in bb:
            k = KIND.get(surf.tag)
            if k is None:
                continue
            ms = surf.find(B + "lod2MultiSurface")
            if ms is None:
                continue
            for rings in polys(ms):
                faces.append([k] + rings)
    if faces:
        lod = 2
    else:
        solid = el.find(B + "lod1Solid")
        if solid is None:
            return None
        for rings in polys(solid):
            faces.append(["?"] + rings)  # 種類は向きで後から決める
    allp = [p for f in faces for p in f[1]]
    if not allp:
        return None
    cx = sum(p[0] for p in allp) / len(allp)
    cy = sum(p[1] for p in allp) / len(allp)
    if abs(cx) > HALF or abs(cy) > HALF:
        return None
    mh = el.find(B + "measuredHeight")
    return {
        "id": el.get(G + "id"),
        "mesh": mesh,
        "h": float(mh.text) if mh is not None else None,
        "lod": lod,
        "faces": faces,
    }


def main(src, dst):
    n = kept = 0
    with open(dst, "w") as fo:
        for fn in sorted(os.listdir(src)):
            if not fn.endswith(".gml"):
                continue
            mesh = fn.split("_")[0]
            for ev, el in ET.iterparse(os.path.join(src, fn), events=("end",)):
                if el.tag != B + "Building":
                    continue
                n += 1
                b = building(el, mesh)
                if b:
                    kept += 1
                    fo.write(json.dumps(b, separators=(",", ":")) + "\n")
                el.clear()
            print(fn, "累計", n, "棟中", kept, "棟を採用", flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
