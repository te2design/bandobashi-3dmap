"""国土地理院の標高タイル(PNG) → 局所座標の格子（5m 間隔）

使い方: python3 -I prep_dem.py <タイル保存フォルダ> <出力.json>
- 5m メッシュ(dem5a)を優先し、欠けている所は 10m メッシュ(dem)で埋める
- 出力: {"step":5, "n":601, "origin":-1500, "h":[...]}  h[j*n+i] は x=origin+i*step, y=origin+j*step
- 標高 PNG の仕様: x = R*65536 + G*256 + B, x<2^23 → x*0.01m, x=2^23 → 無効, それ以外 (x-2^24)*0.01m
"""
import os
import sys
import json
import math
import zlib
import struct
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geo import to_latlon, HALF  # noqa: E402

STEP = 5.0
SRC = [("dem5a_png", 15), ("dem_png", 14)]  # 優先順


def png_rgb(data):
    """8bit RGB(A) の非インターレース PNG を読む（標準ライブラリのみ）"""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat, w = 8, b"", 0
    while pos < len(data):
        ln, typ = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + ln]
        if typ == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
            assert depth == 8 and ctype in (2, 6), (depth, ctype)
            bpp = 3 if ctype == 2 else 4
        elif typ == b"IDAT":
            idat += body
        pos += 12 + ln
    raw = zlib.decompress(idat)
    stride = w * bpp
    rows, prev = [], bytearray(stride)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = bytearray(raw[y * (stride + 1) + 1:(y + 1) * (stride + 1)])
        for i in range(stride):
            a = line[i - bpp] if i >= bpp else 0
            b = prev[i]
            c = prev[i - bpp] if i >= bpp else 0
            if f == 1:
                line[i] = (line[i] + a) & 255
            elif f == 2:
                line[i] = (line[i] + b) & 255
            elif f == 3:
                line[i] = (line[i] + (a + b) // 2) & 255
            elif f == 4:
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                line[i] = (line[i] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        rows.append(line)
        prev = line
    out = []
    for line in rows:
        r = []
        for i in range(w):
            v = line[i * bpp] * 65536 + line[i * bpp + 1] * 256 + line[i * bpp + 2]
            r.append(None if v == 1 << 23 else (v * 0.01 if v < 1 << 23 else (v - (1 << 24)) * 0.01))
        out.append(r)
    return out


class Tiles:
    def __init__(self, cache, name, z):
        self.cache, self.name, self.z, self.mem = cache, name, z, {}

    def get(self, tx, ty):
        k = (tx, ty)
        if k not in self.mem:
            fn = os.path.join(self.cache, f"{self.name}_{self.z}_{tx}_{ty}.png")
            if not os.path.exists(fn):
                url = f"https://cyberjapandata.gsi.go.jp/xyz/{self.name}/{self.z}/{tx}/{ty}.png"
                try:
                    data = urllib.request.urlopen(url, timeout=30).read()
                except Exception:
                    data = b""
                open(fn, "wb").write(data)
            data = open(fn, "rb").read()
            self.mem[k] = png_rgb(data) if data else None
        return self.mem[k]

    def sample(self, lat, lon):
        """双線形補間。無効値を含むなら None"""
        n = 2 ** self.z
        px = (lon + 180) / 360 * n * 256 - 0.5
        s = math.sin(math.radians(lat))
        py = (0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)) * n * 256 - 0.5
        x0, y0 = math.floor(px), math.floor(py)
        fx, fy = px - x0, py - y0
        vals = []
        for dy in (0, 1):
            for dx in (0, 1):
                gx, gy = x0 + dx, y0 + dy
                t = self.get(gx // 256, gy // 256)
                v = t[gy % 256][gx % 256] if t else None
                if v is None:
                    return None
                vals.append(v)
        return (vals[0] * (1 - fx) * (1 - fy) + vals[1] * fx * (1 - fy)
                + vals[2] * (1 - fx) * fy + vals[3] * fx * fy)


def main(cache, dst):
    os.makedirs(cache, exist_ok=True)
    srcs = [Tiles(cache, nm, z) for nm, z in SRC]
    n = int(2 * HALF / STEP) + 1
    h, miss = [], 0
    for j in range(n):
        for i in range(n):
            lat, lon = to_latlon(-HALF + i * STEP, -HALF + j * STEP)
            v = None
            for t in srcs:
                v = t.sample(lat, lon)
                if v is not None:
                    break
            if v is None:
                miss += 1
            h.append(None if v is None else round(v, 2))
        if j % 50 == 0:
            print("行", j, "/", n, flush=True)
    json.dump({"step": STEP, "n": n, "origin": -HALF, "h": h}, open(dst, "w"))
    vv = [x for x in h if x is not None]
    print("完了  欠け", miss, "点  最低", min(vv), "最高", max(vv))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
