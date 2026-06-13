#!/usr/bin/env python3
"""
Route → TCX converter for Ride Simulator
========================================
Converts a route file (.gpx or .fit, with or without elevation) into a TCX file
with synthesized speed suitable as a ride file in the Ride Simulator app.

Input flavors, all handled:
  • Garmin Connect / RideWithGPS FIT course (.fit) — carries real elevation
    (enhanced_altitude). The clean way to get a well-routed path with good grade;
    --reverse handles a course routed backwards down a one-way street.
  • Strava / komoot / RideWithGPS GPX/TCX export — has <ele> per point. Used directly.
  • OSM street-route (tools/route_planner.py, creator="route_planner") — lat/lon
    only, NO elevation. Grade would be flat. This BACK-FILLS elevation by sampling
    the same public AWS Terrain DEM tiles the world bake uses (cached under
    data/tiles/), so an OSM-routed GPX still produces a ride with real hills.

Synthesis model (unchanged from the original):
  • Baseline speed (user-chosen, e.g. 25 km/h)
  • Grade-adjusted: slower uphill, faster downhill (bounded by braking)
  • Smoothed with a short moving average so transitions feel natural
  • Elevation Gaussian-smoothed first to kill GPS/DEM jitter

Output: TCX with <Time>, <Position>, <AltitudeMeters>, <DistanceMeters> for every
trackpoint. Compatible with load_tcx_route() in ride_sim.py.

Usage:
  python gpx_to_tcx.py
    → GUI dialog (auto-fills flat elevation from DEM if data/tiles/ is present)

  python gpx_to_tcx.py input.gpx output.tcx --avg 25 --grade-sensitivity 7
    → command-line mode. DEM back-fill is automatic when the GPX is flat.

  python gpx_to_tcx.py in.gpx out.tcx --dem on    # force DEM even if <ele> exists
  python gpx_to_tcx.py in.gpx out.tcx --dem off   # never touch elevation

Dependencies:
  pip install PySide6 numpy pillow      (numpy/pillow only needed for DEM fill)
"""

import math
import os
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import List, Optional, Tuple

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt


# ─────────────────────────────────────────────────────────────
#  Geometry helpers
# ─────────────────────────────────────────────────────────────

EARTH_R = 6371008.8   # metres, WGS-84 mean radius

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Great-circle distance between two lat/lon points in metres."""
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def gaussian_smooth(xs: List[float], sigma: float) -> List[float]:
    """1-D Gaussian smoothing. sigma in samples."""
    if sigma <= 0:
        return list(xs)
    radius = max(1, int(3 * sigma))
    kernel = [math.exp(-(i * i) / (2 * sigma * sigma))
              for i in range(-radius, radius + 1)]
    s = sum(kernel)
    kernel = [k / s for k in kernel]
    n = len(xs)
    out = [0.0] * n
    for i in range(n):
        acc = 0.0
        wsum = 0.0
        for k_i, k in enumerate(kernel):
            j = i + (k_i - radius)
            if 0 <= j < n:
                acc += xs[j] * k
                wsum += k
        out[i] = acc / wsum if wsum > 0 else xs[i]
    return out


def moving_average(xs: List[float], window: int) -> List[float]:
    """Symmetric moving average with edge clamping."""
    if window <= 1:
        return list(xs)
    half = window // 2
    n = len(xs)
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = sum(xs[lo:hi]) / (hi - lo)
    return out


def densify_track(lat: List[float], lon: List[float], ele: List[float],
                  spacing_m: float
                  ) -> Tuple[List[float], List[float], List[float]]:
    """
    Resample the track to ~uniform `spacing_m` along its length (linear interp of
    lat/lon/ele). Course exports are often sparse (~15–25 m between points); since
    ride_sim derives grade over a SHORT lookahead (~12 m) by snapping to the
    nearest trackpoint, points spaced wider than the lookahead make grade read 0
    (both ends land on the same point). Densifying finer than the lookahead lets
    grade resolve, and smooths the map/ride. spacing_m <= 0 disables.
    """
    n = len(lat)
    if spacing_m <= 0 or n < 2:
        return lat, lon, ele
    d = [0.0] * n
    for i in range(1, n):
        d[i] = d[i - 1] + haversine_m(lat[i - 1], lon[i - 1], lat[i], lon[i])
    total = d[-1]
    if total < spacing_m:
        return lat, lon, ele
    out_lat, out_lon, out_ele = [], [], []
    x = 0.0
    j = 0
    while x <= total:
        while j < n - 2 and d[j + 1] < x:
            j += 1
        span = d[j + 1] - d[j]
        f = 0.0 if span < 1e-9 else (x - d[j]) / span
        out_lat.append(lat[j] + (lat[j + 1] - lat[j]) * f)
        out_lon.append(lon[j] + (lon[j + 1] - lon[j]) * f)
        out_ele.append(ele[j] + (ele[j + 1] - ele[j]) * f)
        x += spacing_m
    out_lat.append(lat[-1]); out_lon.append(lon[-1]); out_ele.append(ele[-1])
    return out_lat, out_lon, out_ele


def distance_smooth(dist_m: List[float], xs: List[float], window_m: float) -> List[float]:
    """
    Smooth xs over a ±window_m/2 *distance* window (not a sample count).

    Point spacing is uneven — dense on switchbacks, sparse on straights — so a
    fixed-sample smoother over- or under-smooths depending on geometry. A
    distance window gives uniform spatial smoothing, which is what's needed to
    dissolve the staircase a coarse DEM leaves in the elevation-vs-distance
    profile (plateaus where the route doubles back through the same DEM cells,
    with sudden risers between). Spreading each riser across the window turns a
    "0% then a huge spike" grade into a sustained, believable one. dist_m must
    be non-decreasing (cumulative distance), so the window bounds advance
    monotonically — O(n).
    """
    if window_m <= 0:
        return list(xs)
    n = len(xs)
    out = [0.0] * n
    half = window_m / 2.0
    j0 = 0
    j1 = 0
    for i in range(n):
        lo = dist_m[i] - half
        hi = dist_m[i] + half
        while j0 < n and dist_m[j0] < lo:
            j0 += 1
        while j1 < n and dist_m[j1] <= hi:
            j1 += 1
        if j1 > j0:
            out[i] = sum(xs[j0:j1]) / (j1 - j0)
        else:
            out[i] = xs[i]
    return out


# ─────────────────────────────────────────────────────────────
#  DEM elevation back-fill (AWS Terrain Tiles, terrarium-encoded)
#
#  Mirrors ride-sim-world/tools/dem_to_heightmap.py: same tile source, same
#  cache naming ({z}_{x}_{y}.png), same terrarium decode. Tiles already cached
#  for a route are read offline; anything missing is fetched once.
# ─────────────────────────────────────────────────────────────

TILE_URL = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


def _default_tiles_dir() -> str:
    """data/tiles next to this tools/ dir — where the world bake caches them."""
    return str(Path(__file__).resolve().parent.parent / "godot" / "data" / "tiles")


def _deg2tile_xy(lat, lon, z):
    """Fractional global tile-pixel coords (slippy-map / Web Mercator)."""
    import numpy as np
    n = 2 ** z
    gx = (np.asarray(lon) + 180.0) / 360.0 * n
    lat_r = np.radians(np.asarray(lat))
    gy = (1.0 - np.arcsinh(np.tan(lat_r)) / math.pi) / 2.0 * n
    return gx, gy


def _fetch_tile(z, x, y, cache):
    """Load a cached terrarium tile as float elevation; fetch+cache if missing."""
    import numpy as np
    from PIL import Image
    fn = os.path.join(cache, f"{z}_{x}_{y}.png")
    if not os.path.exists(fn):
        import urllib.request
        url = TILE_URL.format(z=z, x=x, y=y)
        req = urllib.request.Request(url, headers={"User-Agent": "ride-sim-world/0.1"})
        os.makedirs(cache, exist_ok=True)
        with urllib.request.urlopen(req, timeout=20) as r, open(fn, "wb") as f:
            f.write(r.read())
    rgb = np.asarray(Image.open(fn).convert("RGB"), dtype=np.float64)
    return rgb[:, :, 0] * 256.0 + rgb[:, :, 1] + rgb[:, :, 2] / 256.0 - 32768.0


def dem_elevation(lat: List[float], lon: List[float],
                  tiles_dir: str, zoom: int = 14) -> List[float]:
    """
    Sample absolute DEM elevation (metres) at each lat/lon via bilinear
    interpolation over a mosaic of terrarium tiles. Requires numpy + pillow.
    Reads cached tiles offline; fetches any not yet cached.
    """
    import numpy as np

    gx, gy = _deg2tile_xy(lat, lon, zoom)
    tx = np.floor(gx).astype(int)
    ty = np.floor(gy).astype(int)
    tx0, tx1 = int(tx.min()), int(tx.max())
    ty0, ty1 = int(ty.min()), int(ty.max())

    th = (ty1 - ty0 + 1) * 256
    tw = (tx1 - tx0 + 1) * 256
    mosaic = np.empty((th, tw), dtype=np.float64)
    for j, tyy in enumerate(range(ty0, ty1 + 1)):
        for i, txx in enumerate(range(tx0, tx1 + 1)):
            mosaic[j*256:(j+1)*256, i*256:(i+1)*256] = _fetch_tile(zoom, txx, tyy, tiles_dir)

    px = gx * 256.0 - tx0 * 256.0
    py = gy * 256.0 - ty0 * 256.0
    px = np.clip(px, 0, tw - 1.001)
    py = np.clip(py, 0, th - 1.001)
    x0i = np.floor(px).astype(int); y0i = np.floor(py).astype(int)
    fx = px - x0i; fy = py - y0i
    h00 = mosaic[y0i, x0i];     h10 = mosaic[y0i, x0i + 1]
    h01 = mosaic[y0i + 1, x0i]; h11 = mosaic[y0i + 1, x0i + 1]
    heights = ((h00 * (1 - fx) + h10 * fx) * (1 - fy)
               + (h01 * (1 - fx) + h11 * fx) * fy)
    return [float(h) for h in heights]


def _is_flat(ele: List[float], tol: float = 1.0) -> bool:
    """True if elevation carries no usable relief (missing or near-constant)."""
    if not ele:
        return True
    return (max(ele) - min(ele)) < tol


def maybe_fill_elevation(lat, lon, ele, mode: str, tiles_dir: str, zoom: int
                         ) -> Tuple[List[float], str]:
    """
    Apply DEM back-fill per `mode`: 'auto' (fill only if flat), 'on' (always),
    'off' (never). Returns (elevation, status_message). Never raises — on any
    DEM error it logs and returns the original elevation so a flat ride still
    converts.
    """
    if mode == "off":
        return ele, "DEM fill: off"
    if mode == "auto" and not _is_flat(ele):
        return ele, "DEM fill: skipped (GPX already has elevation)"
    reason = "forced" if mode == "on" else "GPX is flat"
    try:
        filled = dem_elevation(lat, lon, tiles_dir, zoom)
        gain = max(filled) - min(filled)
        return filled, f"DEM fill: applied ({reason}), relief {gain:.0f} m @ zoom {zoom}"
    except ModuleNotFoundError:
        return ele, "DEM fill: SKIPPED — needs numpy + pillow (pip install numpy pillow)"
    except Exception as e:  # noqa: BLE001 — never block conversion on DEM trouble
        return ele, f"DEM fill: SKIPPED — {type(e).__name__}: {e}"


# ─────────────────────────────────────────────────────────────
#  GPX parsing
# ─────────────────────────────────────────────────────────────

def parse_gpx(path: str) -> Tuple[List[float], List[float], List[float]]:
    """
    Parse a GPX file, return (lat, lon, ele) lists.
    Missing elevation becomes 0.0.
    """
    ns = {"g": "http://www.topografix.com/GPX/1/1"}
    tree = ET.parse(path)
    root = tree.getroot()

    # Some GPX files use no namespace or a different one
    if root.tag.startswith("{"):
        ns_uri = root.tag[1:root.tag.index("}")]
        ns = {"g": ns_uri}
        trkpts = root.findall(".//g:trkpt", ns)
    else:
        trkpts = root.findall(".//trkpt")
        ns = {}

    if not trkpts:
        raise RuntimeError("No <trkpt> elements found in GPX.")

    lat, lon, ele = [], [], []
    for tp in trkpts:
        try:
            lat.append(float(tp.get("lat")))
            lon.append(float(tp.get("lon")))
        except (TypeError, ValueError):
            continue
        if ns:
            e = tp.find("g:ele", ns)
        else:
            e = tp.find("ele")
        ele.append(float(e.text) if e is not None and e.text else 0.0)

    if len(lat) < 10:
        raise RuntimeError(f"Too few valid trackpoints ({len(lat)}).")

    return lat, lon, ele


# ─────────────────────────────────────────────────────────────
#  FIT parsing (Garmin / RideWithGPS course or activity .fit)
# ─────────────────────────────────────────────────────────────

def parse_fit(path: str) -> Tuple[List[float], List[float], List[float]]:
    """
    Parse a FIT file's `record` messages, return (lat, lon, ele).

    Handles both course .fit (Garmin Connect / RideWithGPS "FIT Course") and
    recorded-activity .fit — we only read position + altitude and synthesize the
    ride, so the distinction doesn't matter. lat/lon are stored as semicircles
    (deg = semicircle * 180 / 2^31). Prefers enhanced_altitude over altitude.
    Course .fit can be sparse (a Garmin Lombard course is ~49 pts) — that's fine;
    ride_sim and the Godot world both interpolate between points.
    """
    try:
        import fitparse
    except ModuleNotFoundError:
        raise RuntimeError(
            "Reading .fit needs the fitparse package: pip install fitparse")

    SEMI = 180.0 / 2 ** 31
    fit = fitparse.FitFile(path)
    lat, lon, ele = [], [], []
    for m in fit.get_messages("record"):
        d = {f.name: f.value for f in m}
        la, lo = d.get("position_lat"), d.get("position_long")
        if la is None or lo is None:
            continue
        lat.append(la * SEMI)
        lon.append(lo * SEMI)
        e = d.get("enhanced_altitude")
        if e is None:
            e = d.get("altitude")
        ele.append(float(e) if e is not None else 0.0)

    if len(lat) < 5:
        raise RuntimeError(
            f"Too few FIT track points ({len(lat)}) — is this a course/activity "
            f"with GPS records?")
    return lat, lon, ele


def load_track(path: str, reverse: bool = False
               ) -> Tuple[List[float], List[float], List[float]]:
    """Dispatch on extension: .fit -> parse_fit, else GPX. Optionally reverse
    the point order (e.g. a Garmin course routed backwards down a one-way)."""
    ext = Path(path).suffix.lower()
    lat, lon, ele = parse_fit(path) if ext == ".fit" else parse_gpx(path)
    if reverse:
        lat, lon, ele = lat[::-1], lon[::-1], ele[::-1]
    return lat, lon, ele


# ─────────────────────────────────────────────────────────────
#  Speed synthesis
# ─────────────────────────────────────────────────────────────

def synthesize_ride(lat: List[float], lon: List[float], ele: List[float],
                    avg_kmh: float, grade_sensitivity: float,
                    max_downhill_kmh: float = 60.0,
                    min_climb_kmh: float = 6.0,
                    elev_smooth_sigma: float = 3.0,
                    elev_smooth_m: float = 60.0,
                    speed_smooth_window: int = 11
                    ) -> Tuple[List[float], List[float], List[float]]:
    """
    Build time and cumulative-distance arrays from a raw GPX track.

    Returns (time_s, dist_m, ele_smoothed).

    Algorithm:
      1. Cumulative distance from haversine between consecutive points.
      2. Gaussian-smooth elevation to reduce GPS altitude jitter.
      3. Compute local grade over a short window (~30 m).
      4. Map grade → speed via:
            speed = avg * (1 - k*grade)         for uphill
            speed = avg * (1 + k*grade*0.5)     for downhill (milder)
         then clamp to [min_climb_kmh, max_downhill_kmh].
      5. Moving-average the speed profile.
      6. Integrate 1/speed to get time at each point.
      7. Renormalize so total distance / total time = avg exactly.
    """
    n = len(lat)

    # Cumulative distance
    dist_m = [0.0] * n
    for i in range(1, n):
        dist_m[i] = dist_m[i-1] + haversine_m(lat[i-1], lon[i-1], lat[i], lon[i])

    total_dist = dist_m[-1]
    if total_dist < 100:
        raise RuntimeError(f"Route is too short ({total_dist:.0f} m).")

    # Smooth elevation. Distance-based (metres) by default so it dissolves the
    # coarse-DEM staircase uniformly regardless of point spacing; falls back to
    # the sample-count Gaussian if elev_smooth_m is 0.
    if elev_smooth_m and elev_smooth_m > 0:
        ele_s = distance_smooth(dist_m, ele, elev_smooth_m)
    else:
        ele_s = gaussian_smooth(ele, elev_smooth_sigma)

    # Compute grade over ~30 m lookahead
    grade_lookahead_m = 30.0
    grade = [0.0] * n
    for i in range(n):
        j = i
        while j < n - 1 and dist_m[j] - dist_m[i] < grade_lookahead_m:
            j += 1
        dx = dist_m[j] - dist_m[i]
        if dx < 1e-3:
            grade[i] = 0.0
        else:
            grade[i] = 100.0 * (ele_s[j] - ele_s[i]) / dx

    # Grade → speed
    k = grade_sensitivity / 100.0   # grade_sensitivity is "percent slowdown per 1% grade"
    speed_kmh = [0.0] * n
    for i in range(n):
        g = grade[i]
        if g >= 0:
            s = avg_kmh * (1.0 - k * g)
        else:
            # Downhill: milder boost, capped
            s = avg_kmh * (1.0 - k * g * 0.5)
        speed_kmh[i] = max(min_climb_kmh, min(max_downhill_kmh, s))

    # Smooth the speed profile
    speed_kmh = moving_average(speed_kmh, speed_smooth_window)

    # Convert to m/s, integrate dist/speed for time
    speed_mps = [s / 3.6 for s in speed_kmh]
    time_s = [0.0] * n
    for i in range(1, n):
        seg_dist = dist_m[i] - dist_m[i-1]
        avg_v = 0.5 * (speed_mps[i-1] + speed_mps[i])
        if avg_v < 0.1:
            avg_v = 0.1
        time_s[i] = time_s[i-1] + seg_dist / avg_v

    # Renormalize to match target average exactly
    target_total_s = total_dist / (avg_kmh / 3.6)
    if time_s[-1] > 0:
        scale = target_total_s / time_s[-1]
        time_s = [t * scale for t in time_s]

    return time_s, dist_m, ele_s


# ─────────────────────────────────────────────────────────────
#  TCX writer
# ─────────────────────────────────────────────────────────────

NS_TCX = "http://www.garmin.com/xmlschemas/TrainingCenterDatabase/v2"

def write_tcx(out_path: str,
              lat: List[float], lon: List[float],
              ele: List[float], dist_m: List[float], time_s: List[float],
              start_time: Optional[datetime] = None):
    """Write a TCX file compatible with ride_sim's load_tcx_route()."""
    if start_time is None:
        start_time = datetime.now(timezone.utc).replace(microsecond=0)

    ET.register_namespace("", NS_TCX)
    root = ET.Element(f"{{{NS_TCX}}}TrainingCenterDatabase")
    acts = ET.SubElement(root, f"{{{NS_TCX}}}Activities")
    act  = ET.SubElement(acts, f"{{{NS_TCX}}}Activity", Sport="Biking")
    ET.SubElement(act, f"{{{NS_TCX}}}Id").text = \
        start_time.strftime("%Y-%m-%dT%H:%M:%SZ")

    lap = ET.SubElement(act, f"{{{NS_TCX}}}Lap",
                        StartTime=start_time.strftime("%Y-%m-%dT%H:%M:%SZ"))
    ET.SubElement(lap, f"{{{NS_TCX}}}TotalTimeSeconds").text = f"{time_s[-1]:.1f}"
    ET.SubElement(lap, f"{{{NS_TCX}}}DistanceMeters").text = f"{dist_m[-1]:.1f}"

    track = ET.SubElement(lap, f"{{{NS_TCX}}}Track")
    for i in range(len(lat)):
        tp = ET.SubElement(track, f"{{{NS_TCX}}}Trackpoint")
        ts = start_time + timedelta(seconds=time_s[i])
        ET.SubElement(tp, f"{{{NS_TCX}}}Time").text = \
            ts.strftime("%Y-%m-%dT%H:%M:%SZ")
        pos = ET.SubElement(tp, f"{{{NS_TCX}}}Position")
        ET.SubElement(pos, f"{{{NS_TCX}}}LatitudeDegrees").text = f"{lat[i]:.7f}"
        ET.SubElement(pos, f"{{{NS_TCX}}}LongitudeDegrees").text = f"{lon[i]:.7f}"
        ET.SubElement(tp, f"{{{NS_TCX}}}AltitudeMeters").text = f"{ele[i]:.1f}"
        ET.SubElement(tp, f"{{{NS_TCX}}}DistanceMeters").text = f"{dist_m[i]:.1f}"

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(out_path, encoding="utf-8", xml_declaration=True)


# ─────────────────────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────────────────────

DARK = """
QMainWindow, QDialog, QWidget { background: #0d0d14; color: #e0e0e0;
    font-family: sans-serif; font-size: 12px; }
QSlider::groove:horizontal { height: 4px; background: #2a2a40; border-radius: 2px; }
QSlider::handle:horizontal { background: #00e5ff; width: 14px; height: 14px;
    margin: -5px 0; border-radius: 7px; }
QSlider::sub-page:horizontal { background: #007a99; border-radius: 2px; }
QPushButton { background: #1a1a2e; border: 1px solid #2a2a40; border-radius: 4px;
    padding: 6px 14px; color: #e0e0e0; }
QPushButton:hover { background: #00a0bb; color: white; }
QPushButton:disabled { color: #555; }
QLineEdit { background: #1a1a2e; border: 1px solid #2a2a40; border-radius: 4px;
    padding: 4px; color: #e0e0e0; }
"""


class ElevationPreview(QtWidgets.QWidget):
    """Simple QPainter elevation-profile visualization."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.dist_m: List[float] = []
        self.ele_m: List[float] = []
        self.speed_kmh: List[float] = []
        self.setMinimumHeight(160)

    def set_data(self, dist_m, ele_m, speed_kmh):
        self.dist_m = dist_m
        self.ele_m = ele_m
        self.speed_kmh = speed_kmh
        self.update()

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QtGui.QColor(20, 20, 32))

        if len(self.dist_m) < 2:
            p.setPen(QtGui.QColor(120, 120, 140))
            p.drawText(self.rect(), Qt.AlignCenter, "Load a GPX file to preview")
            return

        # Elevation profile (filled area)
        emin, emax = min(self.ele_m), max(self.ele_m)
        erange = max(1.0, emax - emin)
        dmax = self.dist_m[-1]

        elev_poly = [QtCore.QPointF(0, h)]
        for i in range(len(self.dist_m)):
            x = self.dist_m[i] / dmax * w
            y = h * 0.4 + (1 - (self.ele_m[i] - emin) / erange) * h * 0.55
            elev_poly.append(QtCore.QPointF(x, y))
        elev_poly.append(QtCore.QPointF(w, h))

        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QBrush(QtGui.QColor(60, 120, 80, 140)))
        p.drawPolygon(QtGui.QPolygonF(elev_poly))
        p.setPen(QtGui.QPen(QtGui.QColor(120, 200, 150), 1.2))
        p.drawPolyline(QtGui.QPolygonF(elev_poly[1:-1]))

        # Speed profile (cyan line along top half)
        if self.speed_kmh:
            smin = min(self.speed_kmh)
            smax = max(self.speed_kmh)
            srange = max(1.0, smax - smin)
            sp_poly = []
            for i in range(len(self.speed_kmh)):
                x = self.dist_m[i] / dmax * w
                y = 10 + (1 - (self.speed_kmh[i] - smin) / srange) * (h * 0.3)
                sp_poly.append(QtCore.QPointF(x, y))
            p.setPen(QtGui.QPen(QtGui.QColor(0, 229, 255), 1.5))
            p.drawPolyline(QtGui.QPolygonF(sp_poly))

        # Scale labels
        p.setPen(QtGui.QColor(140, 140, 160))
        f = QtGui.QFont("sans-serif", 9)
        p.setFont(f)
        p.drawText(4, 14, f"Speed  {smin:.0f}–{smax:.0f} km/h" if self.speed_kmh else "")
        p.drawText(4, h - 4, f"Elev {emin:.0f}–{emax:.0f} m  ·  {dmax/1000:.1f} km")


class ConverterWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GPX → TCX Converter")
        self.setStyleSheet(DARK)
        self.resize(760, 560)

        self._lat = []
        self._lon = []
        self._ele = []
        self._gpx_path: Optional[str] = None

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        lay = QtWidgets.QVBoxLayout(central)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)

        # File row
        frow = QtWidgets.QHBoxLayout()
        self._path_edit = QtWidgets.QLineEdit()
        self._path_edit.setPlaceholderText("No file loaded")
        self._path_edit.setReadOnly(True)
        browse = QtWidgets.QPushButton("Browse GPX…")
        browse.clicked.connect(self._browse)
        frow.addWidget(self._path_edit, 1)
        frow.addWidget(browse)
        lay.addLayout(frow)

        # Info line
        self._info = QtWidgets.QLabel("—")
        self._info.setStyleSheet("color: #888; padding: 2px 0;")
        lay.addWidget(self._info)

        # Preview
        self._preview = ElevationPreview()
        lay.addWidget(self._preview, 1)

        # Controls
        form = QtWidgets.QFormLayout()
        form.setVerticalSpacing(8)

        self._avg = QtWidgets.QSlider(Qt.Horizontal)
        self._avg.setRange(10, 45); self._avg.setValue(25)
        self._avg_v = QtWidgets.QLabel("25 km/h")
        self._avg.valueChanged.connect(
            lambda v: (self._avg_v.setText(f"{v} km/h"), self._recompute()))
        r1 = QtWidgets.QHBoxLayout()
        r1.addWidget(self._avg, 1); r1.addWidget(self._avg_v)
        form.addRow("Target average speed:", r1)

        self._grade = QtWidgets.QSlider(Qt.Horizontal)
        self._grade.setRange(2, 15); self._grade.setValue(7)
        self._grade_v = QtWidgets.QLabel("7 %/%")
        self._grade.valueChanged.connect(
            lambda v: (self._grade_v.setText(f"{v} %/%"), self._recompute()))
        r2 = QtWidgets.QHBoxLayout()
        r2.addWidget(self._grade, 1); r2.addWidget(self._grade_v)
        form.addRow("Grade sensitivity:", r2)

        # DEM elevation back-fill
        self._dem = QtWidgets.QComboBox()
        self._dem.addItems(["auto (fill if flat)", "on (force DEM)", "off"])
        self._dem.currentIndexChanged.connect(self._reload_elevation)
        form.addRow("DEM elevation fill:", self._dem)

        lay.addLayout(form)

        hint = QtWidgets.QLabel(
            "Grade sensitivity: % speed change per 1% grade. 7 is moderate; "
            "3 = strong rider, 12 = casual.  DEM fill samples public terrain "
            "tiles (data/tiles/) so an OSM street-route with no elevation still "
            "gets real hills.")
        hint.setStyleSheet("color: #666; font-size: 11px;")
        hint.setWordWrap(True)
        lay.addWidget(hint)

        # Save row
        srow = QtWidgets.QHBoxLayout()
        srow.addStretch()
        self._save_btn = QtWidgets.QPushButton("Save TCX…")
        self._save_btn.setEnabled(False)
        self._save_btn.clicked.connect(self._save)
        srow.addWidget(self._save_btn)
        lay.addLayout(srow)

    # ── Events ──

    def _dem_mode(self) -> str:
        return ["auto", "on", "off"][self._dem.currentIndex()]

    def _browse(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open route file", "",
            "Route files (*.gpx *.fit);;GPX (*.gpx);;FIT (*.fit);;All files (*)")
        if not path:
            return
        try:
            lat, lon, ele = load_track(path)
            lat, lon, ele = densify_track(lat, lon, ele, 5.0)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Route parse error", str(e))
            return
        self._lat, self._lon, self._ele_raw = lat, lon, ele
        self._gpx_path = path
        self._path_edit.setText(path)
        self._save_btn.setEnabled(True)
        self._reload_elevation()

    def _reload_elevation(self):
        """(Re)apply the DEM-fill policy to the raw elevation, then recompute."""
        if not self._lat:
            return
        ele, status = maybe_fill_elevation(
            self._lat, self._lon, self._ele_raw,
            self._dem_mode(), _default_tiles_dir(), zoom=14)
        self._ele = ele
        self._dem_status = status
        self._recompute()

    def _recompute(self):
        if not self._lat:
            return
        try:
            time_s, dist_m, ele_s = synthesize_ride(
                self._lat, self._lon, self._ele,
                avg_kmh=self._avg.value(),
                grade_sensitivity=self._grade.value())
        except Exception as e:
            self._info.setText(f"<span style='color:#e77'>Error: {e}</span>")
            return

        self._time_s = time_s
        self._dist_m = dist_m
        self._ele_s = ele_s

        # Compute per-segment speed for preview
        speed_kmh = []
        for i in range(len(time_s)):
            if i == 0:
                speed_kmh.append(self._avg.value())
                continue
            dt = time_s[i] - time_s[i-1]
            dx = dist_m[i] - dist_m[i-1]
            speed_kmh.append(dx / dt * 3.6 if dt > 0 else 0.0)

        self._preview.set_data(dist_m, ele_s, speed_kmh)

        mins = int(time_s[-1] / 60)
        km = dist_m[-1] / 1000
        elev_gain = sum(max(0, self._ele[i] - self._ele[i-1])
                        for i in range(1, len(self._ele)))
        self._info.setText(
            f"{len(self._lat)} points  ·  {km:.2f} km  ·  "
            f"{mins} min  ·  +{elev_gain:.0f} m gain  ·  {self._dem_status}")

    def _save(self):
        if not self._lat:
            return
        default_name = Path(self._gpx_path).with_suffix(".tcx").name
        out, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save TCX file",
            str(Path(self._gpx_path).parent / default_name),
            "TCX files (*.tcx);;All files (*)")
        if not out:
            return
        try:
            write_tcx(out, self._lat, self._lon, self._ele_s,
                      self._dist_m, self._time_s)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Save error", str(e))
            return
        QtWidgets.QMessageBox.information(
            self, "Saved", f"Wrote TCX file:\n{out}")


# ─────────────────────────────────────────────────────────────
#  CLI + entry point
# ─────────────────────────────────────────────────────────────

def cli(argv):
    import argparse
    ap = argparse.ArgumentParser(
        description="Convert a route (.gpx or .fit) to a ride_sim TCX with synthesized speed.")
    ap.add_argument("input", help="Input route file (.gpx or .fit)")
    ap.add_argument("output", help="Output .tcx file")
    ap.add_argument("--avg", type=float, default=25.0,
                    help="Target average speed km/h (default 25)")
    ap.add_argument("--grade-sensitivity", type=float, default=7.0,
                    help="Grade sensitivity, percent slowdown per 1%% grade (default 7)")
    ap.add_argument("--elev-smooth-m", type=float, default=60.0,
                    help="Elevation smoothing window in metres (default 60). Spreads "
                         "the coarse-DEM staircase into a believable grade; 0 = off")
    ap.add_argument("--reverse", action="store_true",
                    help="Reverse point order (e.g. a Garmin course routed backwards "
                         "down a one-way street)")
    ap.add_argument("--resample-m", type=float, default=5.0,
                    help="Densify the track to this spacing in metres (default 5). "
                         "Sparse course exports need this so ride_sim's short grade "
                         "lookahead resolves; 0 = keep original points")
    ap.add_argument("--dem", choices=["auto", "on", "off"], default="auto",
                    help="DEM elevation back-fill: auto=fill only if the route is flat "
                         "(default), on=always, off=never")
    ap.add_argument("--dem-tiles", default=None,
                    help="terrarium tile cache dir (default: ../godot/data/tiles)")
    ap.add_argument("--dem-zoom", type=int, default=14,
                    help="DEM tile zoom (default 14, ~7.6 m/px)")
    args = ap.parse_args(argv)

    print(f"Reading {args.input}…")
    lat, lon, ele = load_track(args.input, reverse=args.reverse)
    relief = max(ele) - min(ele) if ele else 0.0
    print(f"  {len(lat)} track points  ·  source elevation relief {relief:.0f} m"
          f"{'  (reversed)' if args.reverse else ''}")
    if args.resample_m > 0:
        before = len(lat)
        lat, lon, ele = densify_track(lat, lon, ele, args.resample_m)
        print(f"  densified {before} → {len(lat)} pts @ {args.resample_m:g} m spacing")

    tiles_dir = args.dem_tiles or _default_tiles_dir()
    ele, status = maybe_fill_elevation(lat, lon, ele, args.dem, tiles_dir, args.dem_zoom)
    print(f"  {status}")

    time_s, dist_m, ele_s = synthesize_ride(
        lat, lon, ele,
        avg_kmh=args.avg,
        grade_sensitivity=args.grade_sensitivity,
        elev_smooth_m=args.elev_smooth_m)
    print(f"  Total distance: {dist_m[-1]/1000:.2f} km")
    print(f"  Total time:     {time_s[-1]/60:.1f} min")
    print(f"  Avg speed:      {dist_m[-1]/time_s[-1]*3.6:.2f} km/h")
    print(f"  Elevation:      {min(ele_s):.0f}..{max(ele_s):.0f} m")

    write_tcx(args.output, lat, lon, ele_s, dist_m, time_s)
    print(f"Wrote {args.output}")


def main():
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        # Treat extra args as CLI mode if two positional args given
        if len(sys.argv) >= 3:
            cli(sys.argv[1:])
            return
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    w = ConverterWindow()
    w.show()
    app.exec()


if __name__ == "__main__":
    main()
