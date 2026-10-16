"""共通：範囲・座標変換（緯度経度 → 中心からの東西・南北メートル）"""
import math

# 中心：阪東橋駅（Google マップのピン位置）
LAT0, LON0 = 35.437596, 139.625177
HALF = 750.0  # 中心から四辺までの距離 [m]（1.5km 四方）

_A = 6378137.0
_F = 1 / 298.257222101  # GRS80（JGD2011）
_E2 = _F * (2 - _F)


def _ecef(lat, lon, h=0.0):
    p, l = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1 - _E2 * math.sin(p) ** 2)
    return ((n + h) * math.cos(p) * math.cos(l),
            (n + h) * math.cos(p) * math.sin(l),
            (n * (1 - _E2) + h) * math.sin(p))


_X0, _Y0, _Z0 = _ecef(LAT0, LON0)
_sp, _cp = math.sin(math.radians(LAT0)), math.cos(math.radians(LAT0))
_sl, _cl = math.sin(math.radians(LON0)), math.cos(math.radians(LON0))


def to_local(lat, lon):
    """局所接平面（ENU）の東 x・北 y [m]。3km 四方なら誤差は数 mm 程度"""
    x, y, z = _ecef(lat, lon)
    dx, dy, dz = x - _X0, y - _Y0, z - _Z0
    e = -_sl * dx + _cl * dy
    n = -_sp * _cl * dx - _sp * _sl * dy + _cp * dz
    return e, n


def to_latlon(e, n):
    """to_local の逆（反復で解く）"""
    lat, lon = LAT0, LON0
    for _ in range(6):
        e1, n1 = to_local(lat, lon)
        lat += (n - n1) / 111000.0
        lon += (e - e1) / (111000.0 * math.cos(math.radians(lat)))
    return lat, lon


def bbox_latlon():
    """四辺：南・西・北・東（四隅の外接で求める）"""
    corners = [to_latlon(sx * HALF, sy * HALF) for sx in (-1, 1) for sy in (-1, 1)]
    lats = [c[0] for c in corners]
    lons = [c[1] for c in corners]
    return min(lats), min(lons), max(lats), max(lons)


if __name__ == "__main__":
    s, w, n, e = bbox_latlon()
    print(f"南 {s:.6f}  北 {n:.6f}  西 {w:.6f}  東 {e:.6f}")
    for name, (sx, sy) in {"北西": (-1, 1), "北東": (1, 1), "南東": (1, -1), "南西": (-1, -1)}.items():
        la, lo = to_latlon(sx * HALF, sy * HALF)
        print(name, f"{la:.6f}, {lo:.6f}")
