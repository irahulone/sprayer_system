from __future__ import annotations
import json
import numpy as np

R_EARTH = 6_371_000.0  # meters
_EPS = 1e-9
SEAM_TOL_M = 0.05
# ---------------- helpers ----------------

BEARING_CONVENTION = 'CCW'   # set to 'CW' or 'CCW'

@staticmethod
def _wrap360(a):  # normalize degrees into [0,360)
    return float(a % 360.0)

@staticmethod
def _user_to_cw(deg):
    """Convert a bearing in the selected convention to 'CW' internal."""
    deg = _wrap360(deg)
    return deg if BEARING_CONVENTION == 'CW' else _wrap360(360.0 - deg)

@staticmethod
def _cw_to_user(deg):
    """Convert a 'CW' internal bearing back to the selected convention."""
    deg = _wrap360(deg)
    return deg if BEARING_CONVENTION == 'CW' else _wrap360(360.0 - deg)

@staticmethod
def _snap_first_point(XY, prev_last_xy, tol=SEAM_TOL_M):
    if XY is not None and len(XY) and prev_last_xy is not None:
        if np.linalg.norm(XY[0] - prev_last_xy) < tol:
            XY[0] = prev_last_xy
    return XY

@staticmethod
def is_line(seg) -> bool:
    return getattr(seg, 'kind', None) == 'line'

@staticmethod
def is_arc(seg) -> bool:
    return getattr(seg, 'kind', None) == 'arc'

@staticmethod
def _wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

@staticmethod
def _bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from (lat1,lon1) to (lat2,lon2).
       Returns in the SELECTED convention (0°=N, CW+ or CCW+ per switch)."""
    φ1 = np.radians(lat1)
    φ2 = np.radians(lat2)
    dλ = np.radians(lon2 - lon1)
    x = np.sin(dλ) * np.cos(φ2)
    y = np.cos(φ1)*np.sin(φ2) - np.sin(φ1)*np.cos(φ2)*np.cos(dλ)
    θ_cw = _wrap360(np.degrees(np.arctan2(x, y)))  # 0°=N, CW+
    return _cw_to_user(θ_cw)

@staticmethod
def _haversine_m(lat1, lon1, lat2, lon2):
    φ1 = np.radians(lat1)
    φ2 = np.radians(lat2)
    dφ = φ2 - φ1
    dλ = np.radians(lon2 - lon1)
    a = np.sin(dφ/2.0)**2 + np.cos(φ1)*np.cos(φ2)*np.sin(dλ/2.0)**2
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return float(R_EARTH * c)

@staticmethod
def _latlon_to_xy_m(lat, lon, lat0, lon0):
    """Local tangent-plane (ENU-ish) linearization around (lat0,lon0)."""
    φ0 = np.radians(lat0)
    dφ = np.radians(lat - lat0)
    dλ = np.radians(lon - lon0)
    x = R_EARTH * dλ * np.cos(φ0)   # east
    y = R_EARTH * dφ                # north
    return float(x), float(y)

@staticmethod
def _xy_to_bearing_deg(dx, dy):
    """Bearing of ENU vector (dx,dy) in the SELECTED convention."""
    # From ENU to CW-bearing: east=0°, CCW+ → north=90° shift & flip
    θ_east_ccw = np.degrees(np.arctan2(dy, dx))         # 0°=E, CCW+
    θ_cw = _wrap360(90.0 - θ_east_ccw)                  # 0°=N, CW+
    return _cw_to_user(θ_cw)

@staticmethod
def _bearing_to_unit_xy(bdeg: float) -> np.ndarray:
    """Unit ENU vector (dx,dy) for a bearing given in the SELECTED convention."""
    θ_cw = np.radians(_user_to_cw(bdeg))
    # Internal CW mapping: 0°->N=(0,1), 90°->E=(1,0)
    return np.array([np.sin(θ_cw), np.cos(θ_cw)], dtype=float)

@staticmethod
def _left_normal(v: np.ndarray) -> np.ndarray:
    return np.array([-v[1], v[0]], dtype=float)

@staticmethod
def _right_normal(v: np.ndarray) -> np.ndarray:
    return np.array([ v[1],-v[0]], dtype=float)

@staticmethod
def _intersect_lines(p1, d1, p2, d2):
    """
    Intersect parametric lines: p1 + λ d1 == p2 + μ d2.
    Returns (C, λ, μ) or (None, None, None) if parallel/ill-conditioned.
    """
    A = np.array([[d1[0], -d2[0]],
                  [d1[1], -d2[1]]], dtype=float)
    b = np.array([p2[0]-p1[0], p2[1]-p1[1]], dtype=float)
    det = float(np.linalg.det(A))
    if np.abs(det) < _EPS:
        return None, None, None
    lam, mu = np.linalg.solve(A, b)
    C = p1 + lam * d1
    return C, float(lam), float(mu)

@staticmethod
def _fr_to_enu(fwd_m, right_m, heading_deg):
    """
    Body frame (forward,right) to ENU, heading given in the SELECTED convention.
    NOTE: 'right' is defined as clockwise +90° from heading (true body-right),
          independent of the bearing convention.
    """
    h = np.radians(_user_to_cw(float(heading_deg)))
    f = np.array([np.sin(h),  np.cos(h)], dtype=float)   # forward
    r = np.array([np.cos(h), -np.sin(h)], dtype=float)   # right = +90° CW from forward
    d = float(fwd_m) * f + float(right_m) * r
    return float(d[0]), float(d[1])

# ---------- ENU inverse ----------

@staticmethod
def _xy_to_latlon_m(x, y, lat0, lon0):
    lat0 = float(lat0); lon0 = float(lon0)
    lat = lat0 + (y / R_EARTH) * (180.0/np.pi)
    lon = lon0 + (x / (R_EARTH*np.cos(np.radians(lat0)))) * (180.0/np.pi)
    return float(lat), float(lon)

# ---------- sampling helpers ----------

@staticmethod
def _sample_line_gc(seg: 'GeodesicLinePath', step_m=2.0):
    n = max(2, int(np.ceil(seg.length_m() / step_m)))

    def ll_to_xyz(lat, lon):
        latr, lonr = np.radians(lat), np.radians(lon)
        cl = np.cos(latr)
        return np.array([cl*np.cos(lonr), cl*np.sin(lonr), np.sin(latr)], dtype=float)

    A = ll_to_xyz(seg.latA, seg.lonA)
    B = ll_to_xyz(seg.latB, seg.lonB)

    dot = np.clip(np.dot(A, B), -1.0, 1.0)
    omega = np.arccos(dot)

    if omega < 1e-12:
        # endpoints basically identical; fall back to linear
        t = np.linspace(0.0, 1.0, n)
        lat = seg.latA + t*(seg.latB - seg.latA)
        lon = seg.lonA + t*(seg.lonB - seg.lonA)
        return np.column_stack([lat, lon])

    t = np.linspace(0.0, 1.0, n)
    sin_om = np.sin(omega)
    P = (np.sin((1.0 - t)*omega)[:,None]/sin_om)*A + (np.sin(t*omega)[:,None]/sin_om)*B
    P /= np.linalg.norm(P, axis=1, keepdims=True)  # renormalize

    lat = np.degrees(np.arctan2(P[:,2], np.hypot(P[:,0], P[:,1])))
    lon = np.degrees(np.arctan2(P[:,1], P[:,0]))
    return np.column_stack([lat, lon])

@staticmethod
def _sample_arc(seg: CircularArc, step_m=2.0):
    n = max(2, int(np.ceil(seg.length_m() / step_m)))
    if seg.turn_left:
        total = (seg.end_angle - seg.start_angle) % (2*np.pi)
        ang = seg.start_angle + np.linspace(0.0, total, n)
    else:
        total = (seg.start_angle - seg.end_angle) % (2*np.pi)
        ang = seg.start_angle - np.linspace(0.0, total, n)
    x = seg.cx + seg.radius*np.cos(ang)
    y = seg.cy + seg.radius*np.sin(ang)
    # anchor is seg.lat0/lon0
    ll = [ _xy_to_latlon_m(xi, yi, seg.lat0, seg.lon0) for xi, yi in zip(x, y) ]
    return np.asarray(ll, dtype=float)

@staticmethod
def _sample_spline_xy(lat0, lon0, xy, step_m=2.0):
    # resample polyline to ~step_m spacing (optional; direct use also ok)
    d = np.diff(xy, axis=0)
    seglen = np.hypot(d[:,0], d[:,1])
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    if s[-1] <= 0:
        pts = xy
    else:
        n = max(2, int(np.ceil(s[-1] / step_m)))
        ss = np.linspace(0.0, s[-1], n)
        # linear interp on each coord
        x = np.interp(ss, s, xy[:,0])
        y = np.interp(ss, s, xy[:,1])
        pts = np.column_stack([x, y])
    return np.asarray([_xy_to_latlon_m(px, py, lat0, lon0) for px, py in pts], dtype=float)

# ---------- GeoJSON writers ----------

@staticmethod
def path_to_geojson(path: Path, step_m=2.0, split_by_segments=True):
    """
    Returns a FeatureCollection dict.
    - SpinePath: groups contiguous points by id → MultiLineString or many LineStrings.
    - MultiSegmentPath: exports each segment as its own LineString.
    - Single segments: one LineString.
    """
    def linefeat(coords, props):
        return {"type":"Feature",
                "properties": props,
                "geometry":{"type":"LineString","coordinates":[[lon,lat] for lat,lon in coords]}}

    fc = {"type":"FeatureCollection","features":[]}
    if isinstance(path, SpinePath):
        ids = path.seg_ids_per_point
        xy  = path.xy
        lat0, lon0 = path.lat0, path.lon0
        runs = []
        start = 0
        for i in range(1, len(ids)+1):
            if i == len(ids) or ids[i] != ids[start]:
                runs.append((ids[start], xy[start:i]))
                start = i
        for pid, chunk in runs:
            # exact endpoints — no per-run resampling
            coords = np.asarray([_xy_to_latlon_m(px, py, lat0, lon0) for px,py in chunk], dtype=float)
            fc["features"].append(linefeat(coords, {"id": pid, "kind": "spine"}))
        return fc

    if isinstance(path, MultiSegmentPath):
        for s in path.segs:
            if isinstance(s, GeodesicLinePath):
                coords = _sample_line_gc(s, step_m=step_m)
            elif isinstance(s, CircularArc):
                coords = _sample_arc(s, step_m=step_m)
            elif isinstance(s, SplineFilletPath):
                coords = _sample_spline_xy(s.lat0, s.lon0, s.xy, step_m=step_m)
            else:
                # last resort: try generic sampling via distance_to_goal marching, or skip
                continue
            fc["features"].append(linefeat(coords, {"id": getattr(s,'id',None),
                                                    "kind": getattr(s,'kind','unknown')}))
        return fc

    # single segment
    s = path
    if isinstance(s, GeodesicLinePath):
        coords = _sample_line_gc(s, step_m=step_m)
    elif isinstance(s, CircularArc):
        coords = _sample_arc(s, step_m=step_m)
    elif isinstance(s, SplineFilletPath):
        coords = _sample_spline_xy(s.lat0, s.lon0, s.xy, step_m=step_m)
    else:
        return fc
    fc["features"].append(linefeat(coords, {"id": getattr(s,'id',None),
                                            "kind": getattr(s,'kind','unknown')}))
    return fc

@staticmethod
def save_geojson(obj, path):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


@staticmethod
def _resolve_point(pt, anchors, lat0, lon0, heading0_deg, default_ref='prev'):
    """
    Resolve a point to (lat, lon).
      Absolute:
        {"lat":..,"lon":..} (+ optional dx_m/dy_m and/or fwd/right)
      Relative to an anchor (default 'prev'):
        {"ref":"name", dx_m/dy_m | fwd_m/right_m | range_m+{bearing_deg|rel_bearing_deg} | dlat_deg/dlon_deg}
        {"ref":"name"}  -> anchor itself

      Optional "frame": "initial" (default) or "local" to use anchor's heading.
    """
    # Absolute dict (with optional offsets)
    if isinstance(pt, dict) and 'lat' in pt and 'lon' in pt:
        base_lat, base_lon = float(pt['lat']), float(pt['lon'])
        dx_e, dy_n = float(pt.get('dx_m', 0.0)), float(pt.get('dy_m', 0.0))
        fwd = float(pt.get('fwd_m', pt.get('forward_m', pt.get('fx_m', 0.0))))
        rgt = float(pt.get('right_m', pt.get('fy_m', 0.0)))
        if fwd or rgt:
            dxe, dyn = _fr_to_enu(fwd, rgt, heading0_deg)
            dx_e += dxe; dy_n += dyn
        if dx_e or dy_n:
            return _xy_to_latlon_m(dx_e, dy_n, base_lat, base_lon)
        return base_lat, base_lon

    # Absolute list/tuple (strict) — only [lat, lon]
    if isinstance(pt, (list, tuple)):
        if len(pt) != 2:
            raise ValueError(f"Absolute list must be [lat, lon], got {pt!r}")
        return float(pt[0]), float(pt[1])

    if not isinstance(pt, dict):
        raise ValueError(f"Unsupported point spec: {pt!r}")

    # Resolve reference anchor
    ref = pt.get('ref', default_ref)
    if ref not in anchors:
        raise KeyError(f"Unknown anchor '{ref}'. Have {list(anchors.keys())}.")
    base_lat, base_lon = anchors[ref]

    # Which heading frame?
    frame = str(pt.get('frame', 'initial')).lower()
    ref_heading = anchors.get(f'{ref}_heading_deg', heading0_deg) if frame == 'local' else heading0_deg

    # ENU meters
    if 'dx_m' in pt or 'dy_m' in pt:
        dx = float(pt.get('dx_m', 0.0)); dy = float(pt.get('dy_m', 0.0))
        return _xy_to_latlon_m(dx, dy, base_lat, base_lon)

    # Body-frame meters (forward/right)
    if any(k in pt for k in ('fwd_m','forward_m','fx_m','right_m','fy_m')):
        fwd = float(pt.get('fwd_m', pt.get('forward_m', pt.get('fx_m', 0.0))))
        rgt = float(pt.get('right_m', pt.get('fy_m', 0.0)))
        dx, dy = _fr_to_enu(fwd, rgt, ref_heading)
        return _xy_to_latlon_m(dx, dy, base_lat, base_lon)

    # Polar offsets
    if 'range_m' in pt and ('bearing_deg' in pt or 'rel_bearing_deg' in pt):
        rng = float(pt['range_m'])
        bdeg = float(pt['bearing_deg']) if 'bearing_deg' in pt else (ref_heading + float(pt['rel_bearing_deg']))
        ux, uy = _bearing_to_unit_xy(bdeg)
        return _xy_to_latlon_m(rng*ux, rng*uy, base_lat, base_lon)

    # Degree offsets
    if 'dlat_deg' in pt or 'dlon_deg' in pt:
        return float(base_lat + pt.get('dlat_deg', 0.0)), float(base_lon + pt.get('dlon_deg', 0.0))

    # Anchor itself
    return float(base_lat), float(base_lon)

@staticmethod
def _resolve_points_list(points_list, anchors, lat0, lon0, heading0_deg):
    return np.asarray([_resolve_point(p, anchors, lat0, lon0, heading0_deg) for p in points_list], dtype=float)





# ---------------- interface ----------------

class Path:
    def along_fraction(self, lat, lon) -> float: ...
    def cross_track_error(self, lat, lon) -> float: ...
    def heading_at(self, lat, lon) -> float: ...
    def distance_to_goal(self, lat, lon) -> float: ...
    def length_m(self) -> float: ...
    def __repr__(self):
        k = getattr(self, 'kind', 'path')
        i = getattr(self, 'id', None)
        try:
            L = self.length_m()
        except Exception:
            L = float('nan')
        return f"<{self.__class__.__name__} id={i!r} kind={k} len_m={L:.3f}>"

# ---------------- segments ----------------

class GeodesicLinePath(Path):
    """Great-circle segment from A->B. Uses constant initial bearing for heading."""
    def __init__(self, latA, lonA, latB, lonB, seg_id = None):
        self.id = seg_id
        self.kind = 'line'
        self.latA, self.lonA = float(latA), float(lonA)
        self.latB, self.lonB = float(latB), float(lonB)
        self._θAB = np.radians(_bearing_deg(latA, lonA, latB, lonB))
        self._δAB = _haversine_m(latA, lonA, latB, lonB) / R_EARTH
        self._len = float(R_EARTH * self._δAB)

    def __repr__(self):
        return (f"<GeodesicLinePath id={self.id!r} kind=line "
                f"A=({self.latA:.6f},{self.lonA:.6f}) "
                f"B=({self.latB:.6f},{self.lonB:.6f}) "
                f"len_m={self._len:.3f}>")

    def along_fraction(self, latP, lonP):
        δAP = _haversine_m(self.latA, self.lonA, latP, lonP) / R_EARTH
        θAP = np.radians(_bearing_deg(self.latA, self.lonA, latP, lonP))
        δAT = np.arctan2(np.sin(δAP)*np.cos(θAP - self._θAB), np.cos(δAP))
        if self._δAB <= _EPS:
            return 1.0
        t = np.clip(δAT / self._δAB, 0.0, 1.0)
        return float(t)

    def cross_track_error(self, latP, lonP):
        δAP = _haversine_m(self.latA, self.lonA, latP, lonP) / R_EARTH
        θAP = np.radians(_bearing_deg(self.latA, self.lonA, latP, lonP))
        xt = np.arcsin(np.sin(δAP) * np.sin(θAP - self._θAB)) * R_EARTH
        return float(xt)

    def heading_at(self, lat, lon):
        return float(np.degrees(self._θAB) % 360.0)

    def distance_to_goal(self, lat, lon):
        return float(_haversine_m(lat, lon, self.latB, self.lonB))

    def length_m(self):
        return float(self._len)

class SplineFilletPath(Path):
    """A single CR-sampled polyline with the Path interface (id/kind-aware)."""
    def __init__(self, lat0, lon0, xy_points, seg_id=None, kind='cr'):
        self.kind = kind
        self.id = seg_id
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.xy = np.asarray(xy_points, dtype=float)  # shape (N,2)

        d = np.diff(self.xy, axis=0)
        seglen = np.hypot(d[:,0], d[:,1])
        self.s = np.concatenate([[0.0], np.cumsum(seglen)])
        self.total = float(self.s[-1]) if self.s.size else 1.0

    def __repr__(self):
        return (f"<SplineFilletPath id={self.id!r} kind={self.kind} "
                f"pts={len(self.xy)} len_m={self.total:.3f}>")

    @staticmethod
    def _nearest_on_polyline(P, S, x, y):
        dP = np.diff(P, axis=0)
        L  = np.hypot(dP[:,0], dP[:,1]) + 1e-12
        v  = np.column_stack([x - P[:-1,0], y - P[:-1,1]])
        u  = (v[:,0]*dP[:,0] + v[:,1]*dP[:,1]) / (L**2)
        uC = np.clip(u, 0.0, 1.0)
        proj = P[:-1] + dP * uC[:,None]
        k = int(np.argmin(np.sum((proj - np.array([x,y]))**2, axis=1)))
        px, py = proj[k]
        s_along = float(S[k] + uC[k]*L[k])
        tan = dP[k] / L[k]
        err = np.array([x - px, y - py])
        z = tan[0]*err[1] - tan[1]*err[0]
        signed_cte = float(np.sign(z) * np.hypot(err[0], err[1]))
        hdg_deg = float(_xy_to_bearing_deg(tan[0], tan[1]))
        return s_along, px, py, signed_cte, hdg_deg

    # ---- Path interface ----
    def along_fraction(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        s_along, *_ = self._nearest_on_polyline(self.xy, self.s, x, y)
        if self.total <= _EPS: return 1.0
        return float(np.clip(s_along / self.total, 0.0, 1.0))

    def cross_track_error(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, cte, _ = self._nearest_on_polyline(self.xy, self.s, x, y)
        return float(cte)

    def heading_at(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, _, hdg = self._nearest_on_polyline(self.xy, self.s, x, y)
        return float(hdg)

    def distance_to_goal(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        s_along, *_ = self._nearest_on_polyline(self.xy, self.s, x, y)
        return float(max(0.0, self.total - s_along))

    def length_m(self):
        return float(self.total)

    # ---- builders for convenience ----
    @classmethod
    def from_bearings(cls, latA, lonA, in_brg_deg, latB, lonB, out_brg_deg,
                      lat0=None, lon0=None, seg_id=None, samples=40, dist_hint_m=None, alpha=0.5):
        lat0 = float(lat0 if lat0 is not None else latA)
        lon0 = float(lon0 if lon0 is not None else lonA)
        # reuse the CR fallback we used in SpinePath
        xA, yA = _latlon_to_xy_m(latA, lonA, lat0, lon0)
        xB, yB = _latlon_to_xy_m(latB, lonB, lat0, lon0)
        A = np.array([xA, yA], dtype=float); B = np.array([xB, yB], dtype=float)
        AB = B - A; gap = float(np.hypot(AB[0], AB[1]))
        d = dist_hint_m if dist_hint_m is not None else 0.33 * gap
        d = float(np.clip(d, 2.0, 50.0))
        uA = _bearing_to_unit_xy(float(in_brg_deg))
        uB = _bearing_to_unit_xy(float(out_brg_deg))
        A1 = A + d * uA
        B1 = B - d * uB
        if gap < 1e-3:
            XY = np.linspace(A, B, max(2, samples))
        else:
            ctrl = np.vstack([A, A1, B1, B])
            XY = SpinePath._catmull_rom(ctrl, samples_per_segment=max(2, samples), alpha=float(alpha))
        return cls(lat0, lon0, XY, seg_id=seg_id, kind='cr')

    @classmethod
    def from_line(cls, latA, lonA, latB, lonB, lat0=None, lon0=None, seg_id=None, samples=10):
        lat0 = float(lat0 if lat0 is not None else latA)
        lon0 = float(lon0 if lon0 is not None else lonA)
        x1, y1 = _latlon_to_xy_m(latA, lonA, lat0, lon0)
        x2, y2 = _latlon_to_xy_m(latB, lonB, lat0, lon0)
        t = np.linspace(0.0, 1.0, max(2, samples))
        XY = np.column_stack([x1 + t*(x2-x1), y1 + t*(y2-y1)])
        return cls(lat0, lon0, XY, seg_id=seg_id, kind='line')

class CircularArc(Path):
    """
    One circular arc in a local ENU frame.
    Build with:
      - CircularArc.from_three_points(A, M, B, seg_id=None)
      - CircularArc.from_bearings(A, in_brg_deg, B, out_brg_deg, turn='auto'|'left'|'right', seg_id=None)
    """
    def __init__(self, lat0, lon0, cx, cy, radius,
                 start_angle, end_angle, turn_left, seg_id=None,
                 latA=None, lonA=None, latB=None, lonB=None):
        self.kind = 'arc'
        self.id = seg_id
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.cx, self.cy = float(cx), float(cy)
        self.radius = float(radius)
        self.start_angle = float(start_angle)
        self.end_angle   = float(end_angle)
        self.turn_left   = bool(turn_left)
        self.latA = float(latA) if latA is not None else None
        self.lonA = float(lonA) if lonA is not None else None
        self.latB = float(latB) if latB is not None else None
        self.lonB = float(lonB) if lonB is not None else None

        if self.turn_left:
            delta_angle = (self.end_angle - self.start_angle) % (2*np.pi)
        else:
            delta_angle = (self.start_angle - self.end_angle) % (2*np.pi)
        self.delta_angle = float(delta_angle)
        self._len = float(np.abs(self.radius * self.delta_angle))

    # ---------- constructors ----------
    @classmethod
    def from_three_points(cls, latA, lonA, latM, lonM, latB, lonB, seg_id=None):
        lat0, lon0 = float(latA), float(lonA)
        xA, yA = _latlon_to_xy_m(latA, lonA, lat0, lon0)
        xM, yM = _latlon_to_xy_m(latM, lonM, lat0, lon0)
        xB, yB = _latlon_to_xy_m(latB, lonB, lat0, lon0)

        A = np.array([[2.0*(xM-xA), 2.0*(yM-yA)],
                      [2.0*(xB-xA), 2.0*(yB-yA)]], dtype=float)
        b = np.array([xM**2 - xA**2 + yM**2 - yA**2,
                      xB**2 - xA**2 + yB**2 - yA**2], dtype=float)
        det = float(np.linalg.det(A))
        if np.abs(det) < _EPS:
            raise ValueError("Arc undefined: points nearly colinear.")
        cx, cy = np.linalg.solve(A, b)
        radius = np.hypot(xA - cx, yA - cy)

        def ang(x, y): return np.arctan2(y - cy, x - cx)
        aA, aM, aB = ang(xA,yA), ang(xM,yM), ang(xB,yB)

        ccw_span = (aB - aA) % (2*np.pi)
        on_ccw   = (0.0 <= (aM - aA) % (2*np.pi) <= ccw_span)
        cw_span  = (aA - aB) % (2*np.pi)
        on_cw    = (0.0 <= (aA - aM) % (2*np.pi) <= cw_span)
        turn_left = on_ccw if (on_ccw ^ on_cw) else bool(ccw_span <= cw_span)

        return cls(lat0, lon0, cx, cy, radius, aA, aB, turn_left,
                   seg_id=seg_id, latA=latA, lonA=lonA, latB=latB, lonB=lonB)

    @classmethod
    def from_bearings(cls, latA, lonA, in_brg_deg, latB, lonB, out_brg_deg, turn='auto', seg_id=None):
        lat0, lon0 = float(latA), float(lonA)
        xA, yA = _latlon_to_xy_m(latA, lonA, lat0, lon0)
        xB, yB = _latlon_to_xy_m(latB, lonB, lat0, lon0)
        Axy = np.array([xA, yA], dtype=float)
        Bxy = np.array([xB, yB], dtype=float)

        tA = _bearing_to_unit_xy(float(in_brg_deg))
        tB = _bearing_to_unit_xy(float(out_brg_deg))

        def try_side(side):
            nA = _left_normal(tA) if side == 'left' else _right_normal(tA)
            nB = _left_normal(tB) if side == 'left' else _right_normal(tB)
            C, lam, mu = _intersect_lines(Axy, nA, Bxy, nB)

            if C is None:
                # --- Degenerate: normals are colinear (parallel or anti-parallel).
                # Handle semicircle case where an exact solution exists.
                na = nA / (np.hypot(nA[0], nA[1]) or 1.0)
                nb = nB / (np.hypot(nB[0], nB[1]) or 1.0)

                # If normals point along the same line, check if A->B projects on it.
                if np.allclose(nb, na, atol=1e-9) or np.allclose(nb, -na, atol=1e-9):
                    delta = Bxy - Axy
                    proj = float(delta[0]*na[0] + delta[1]*na[1])  # signed separation along the normal line
                    lam = mu = 0.5 * proj
                    if lam > _EPS:  # need positive distances to the center
                        C = Axy + lam * na
                    else:
                        return None
                else:
                    return None

            # from here on, C, lam, mu are valid
            if lam <= _EPS or mu <= _EPS:
                return None

            radius = 0.5 * (lam + mu)
            aA = np.arctan2(yA - C[1], xA - C[0])
            aB = np.arctan2(yB - C[1], xB - C[0])
            if side == 'left':
                d = (aB - aA) % (2*np.pi)     # CCW
            else:
                d = -((aA - aB) % (2*np.pi))  # CW

            arc_len = float(np.abs(radius * d))
            return dict(C=C, radius=float(radius),
                        aA=float(aA), aB=float(aB),
                        turn_left=(side=='left'),
                        arc_len=arc_len)


        choice = None
        if turn in ('left','right'):
            choice = try_side(turn)
            if choice is None:
                raise ValueError(f"Infeasible {turn}-turn arc for given endpoints & bearings.")
        else:
            L, R = try_side('left'), try_side('right')
            cand = [c for c in (L, R) if c is not None]
            if not cand:
                raise ValueError("Infeasible arc: normals do not meet on +side for either turn.")
            choice = min(cand, key=lambda c: c['arc_len'])

        C = choice['C']
        return cls(lat0, lon0, C[0], C[1], choice['radius'],
                   choice['aA'], choice['aB'], choice['turn_left'],
                   seg_id=seg_id, latA=latA, lonA=lonA, latB=latB, lonB=lonB)

    # ---------- Path interface ----------
    def _project(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        vx, vy = x - self.cx, y - self.cy
        norm = np.hypot(vx, vy) or 1.0
        px, py = self.cx + self.radius * vx / norm, self.cy + self.radius * vy / norm
        a = np.arctan2(py - self.cy, px - self.cx)
        if self.turn_left:
            span = (a - self.start_angle) % (2*np.pi)
            span_end = (self.end_angle - self.start_angle) % (2*np.pi)
            a = (self.start_angle + np.minimum(np.maximum(span, 0.0), span_end)) % (2*np.pi)
        else:
            span = (self.start_angle - a) % (2*np.pi)
            span_end = (self.start_angle - self.end_angle) % (2*np.pi)
            a = (self.start_angle - np.minimum(np.maximum(span, 0.0), span_end)) % (2*np.pi)
        px, py = self.cx + self.radius*np.cos(a), self.cy + self.radius*np.sin(a)
        return float(px), float(py), float(a), float(x), float(y)

    def along_fraction(self, lat, lon):
        _, _, a, *_ = self._project(lat, lon)
        if self.turn_left:
            done = (a - self.start_angle) % (2*np.pi)
            total = (self.end_angle - self.start_angle) % (2*np.pi)
        else:
            done = (self.start_angle - a) % (2*np.pi)
            total = (self.start_angle - self.end_angle) % (2*np.pi)
        if total <= _EPS: return 1.0
        return float(np.clip(done / total, 0.0, 1.0))

    def cross_track_error(self, lat, lon):
        px, py, a, x, y = self._project(lat, lon)
        err = np.array([x - px, y - py], dtype=float)
        tan = np.array([-np.sin(a), np.cos(a)], dtype=float) if self.turn_left else np.array([np.sin(a), -np.cos(a)], dtype=float)
        z = tan[0]*err[1] - tan[1]*err[0]
        sign = -1.0 if z > 0 else (1.0 if z < 0 else 0.0)
        return float(sign * np.linalg.norm(err))

    def heading_at(self, lat, lon):
        _, _, a, *_ = self._project(lat, lon)
        dx, dy = (-np.sin(a), np.cos(a)) if self.turn_left else (np.sin(a), -np.cos(a))
        return float(_xy_to_bearing_deg(dx, dy))

    def distance_to_goal(self, lat, lon):
        _, _, a, x, y = self._project(lat, lon)
        rem_angle = (self.end_angle - a) % (2*np.pi) if self.turn_left else (a - self.end_angle) % (2*np.pi)
        along = np.abs(self.radius * rem_angle)
        radial = np.abs(np.hypot(x - self.cx, y - self.cy) - self.radius)
        return float(along + radial)

    def length_m(self):
        return float(self._len)


class SpinePath(Path):
    """
    Sample-based path made from a mix of primitives.
    Use SpinePath.from_spec(...) to build from a JSON-like spec.
    """
    def __init__(self, lat0, lon0, xy_points, seg_ids_per_point, seg_kinds_per_point):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.xy = np.asarray(xy_points, dtype=float)        # shape (N,2)
        self.seg_ids_per_point = list(seg_ids_per_point)    # len N
        self.seg_kinds_per_point = list(seg_kinds_per_point)

        # cumulative arclength
        diffs = np.diff(self.xy, axis=0)
        seglen = np.hypot(diffs[:,0], diffs[:,1])
        self.s = np.concatenate([[0.0], np.cumsum(seglen)])
        self.total = float(self.s[-1]) if self.s.size else 1.0

    def __repr__(self):
        return (f"<SpinePath pts={len(self.xy)} "
                f"segments={len(set(self.seg_ids_per_point))} len_m={self.total:.3f}>")

    # ---------- builders ----------
    @staticmethod
    def _catmull_rom(points, samples_per_segment=20, alpha=0.5):
        """
        Numerically robust Catmull–Rom with centripetal parameterization.
        - Handles degenerate/duplicate points.
        - Uses phantom endpoints to avoid t1==t0 / t3==t2.
        """
        P = np.asarray(points, dtype=float)
        n = len(P)
        if n <= 1:
            return P.copy()
        if n == 2:
            # just a line
            t = np.linspace(0.0, 1.0, max(2, samples_per_segment))
            return np.column_stack([P[0] + t*(P[1]-P[0])])

        eps = 1e-6
        out = []

        def tj(ti, pa, pb):
            # strictly increasing parameter
            return float(ti + (np.linalg.norm(pb - pa)**alpha + eps))

        def safe_lerp(pa, pb, ta, tb, t):
            dt = (tb - ta)
            if abs(dt) < eps:
                return 0.5*(pa + pb)
            w = (t - ta) / dt
            return (1.0 - w)*pa + w*pb

        for i in range(n - 1):
            p1 = P[i]
            p2 = P[i+1]
            # phantom endpoints to avoid duplicates
            p0 = P[i-1] if i > 0 else (2.0*p1 - p2)
            p3 = P[i+2] if i+2 < n else (2.0*p2 - p1)

            t0 = 0.0
            t1 = tj(t0, p0, p1)
            t2 = tj(t1, p1, p2)
            t3 = tj(t2, p2, p3)

            # sample only within [t1, t2)
            ts = np.linspace(t1, t2, max(2, samples_per_segment), endpoint=False)
            for t in ts:
                A1 = safe_lerp(p0, p1, t0, t1, t)
                A2 = safe_lerp(p1, p2, t1, t2, t)
                A3 = safe_lerp(p2, p3, t2, t3, t)
                B1 = safe_lerp(A1, A2, t0, t2, t)
                B2 = safe_lerp(A2, A3, t1, t3, t)
                C  = safe_lerp(B1, B2, t1, t2, t)
                out.append(C)

            # include knot p2 at the end of last segment
            if i == n - 2:
                out.append(p2)

        return np.asarray(out, dtype=float)



    @classmethod
    def from_spec(cls, spec, default_samples=24):

        def _tup(p): return f"({p[0]:.7f},{p[1]:.7f})"  # pretty lat/lon

        trace = bool(spec.get("trace", False))
        lat0 = float(spec["lat0"]); lon0 = float(spec["lon0"])
        heading0_deg = float(spec.get("initial_bearing_deg", 0.0))   # NEW
        defaults = dict(spec.get("defaults", {}))

        anchors = {"origin": (lat0, lon0), "prev": (lat0, lon0)}

        xy_all = []; ids_all = []; kinds_all = []

        for piece in spec["pieces"]:
            ptype = piece["type"].strip().lower()
            pid = piece.get("id", None)
            samples = int(piece.get("samples", default_samples))
            end_hdg = None

            if ptype == "anchor":
                alat, alon = _resolve_point(piece.get("at", {"ref":"prev"}), anchors, lat0, lon0, heading0_deg)
                anchors[piece["id"]] = (alat, alon)
                anchors["prev"] = (alat, alon)

                if trace:
                    latp, lonp = anchors["prev"]
                    if np.isclose(_haversine_m(latp, lonp, lat0, lon0), 0.0, atol=1e-6):
                        print(f"!! WARNING: after piece {pid!r}, anchors['prev'] is still origin {_tup((lat0,lon0))}")
                continue

            if ptype == "line":
                if "start_lat" in piece and "start_lon" in piece:
                    slat, slon = float(piece["start_lat"]), float(piece["start_lon"])
                else:
                    slat, slon = _resolve_point(piece.get("start", {"ref":"prev"}), anchors, lat0, lon0, heading0_deg)

                if "target_lat" in piece and "target_lon" in piece:
                    tlat, tlon = float(piece["target_lat"]), float(piece["target_lon"])
                else:
                    tlat, tlon = _resolve_point(piece["target"], anchors, lat0, lon0, heading0_deg)

                x1, y1 = _latlon_to_xy_m(slat, slon, lat0, lon0)
                x2, y2 = _latlon_to_xy_m(tlat, tlon, lat0, lon0)
                ts = np.linspace(0.0, 1.0, max(2, samples))
                XY = np.column_stack([x1 + ts*(x2-x1), y1 + ts*(y2-y1)])
                if xy_all and len(XY) >= 1:
                    XY = _snap_first_point(XY, xy_all[-1][-1])
                XY[-1] = np.array(_latlon_to_xy_m(tlat, tlon, lat0, lon0), dtype=float)
                xy_all.append(XY); ids_all += [pid]*len(XY); kinds_all += ['line']*len(XY)
                if trace:
                    print(f"[line {pid}] start={_tup((slat,slon))} -> target={_tup((tlat,tlon))}")
                anchors["prev"] = (tlat, tlon)
                end_hdg = _bearing_deg(slat, slon, tlat, tlon)
                if trace:
                    latp, lonp = anchors["prev"]
                    if np.isclose(_haversine_m(latp, lonp, lat0, lon0), 0.0, atol=1e-6):
                        print(f"!! WARNING: after piece {pid!r}, anchors['prev'] is still origin {_tup((lat0,lon0))}")

            elif ptype == "arc":
                # --- three-point arc ---
                if ("via_lat" in piece and "via_lon" in piece) or ("via" in piece):
                    if "start_lat" in piece and "start_lon" in piece:
                        slat, slon = float(piece["start_lat"]), float(piece["start_lon"])
                    else:
                        slat, slon = _resolve_point(piece.get("start", {"ref":"prev"}),
                                                    anchors, lat0, lon0, heading0_deg)

                    if "target_lat" in piece and "target_lon" in piece:
                        tlat, tlon = float(piece["target_lat"]), float(piece["target_lon"])
                    else:
                        tlat, tlon = _resolve_point(piece["target"],
                                                    anchors, lat0, lon0, heading0_deg)

                    if "via_lat" in piece and "via_lon" in piece:
                        vlat, vlon = float(piece["via_lat"]), float(piece["via_lon"])
                    else:
                        vlat, vlon = _resolve_point(piece["via"],
                                                    anchors, lat0, lon0, heading0_deg)
                    arc = CircularArc.from_three_points(slat, slon, vlat, vlon, tlat, tlon, seg_id=pid)
                    total = (arc.end_angle - arc.start_angle) % (2*np.pi) if arc.turn_left else (arc.start_angle - arc.end_angle) % (2*np.pi)
                    ang = arc.start_angle + np.linspace(0.0, total, max(2, samples)) if arc.turn_left else \
                          arc.start_angle - np.linspace(0.0, total, max(2, samples))
                    x = arc.cx + arc.radius*np.cos(ang)
                    y = arc.cy + arc.radius*np.sin(ang)

                    ll = np.array([_xy_to_latlon_m(xx, yy, arc.lat0, arc.lon0) for xx, yy in zip(x, y)], dtype=float)
                    XY_spine = np.array([_latlon_to_xy_m(lat, lon, lat0, lon0) for lat, lon in ll], dtype=float)

                    # seam snap + force exact target
                    if xy_all and len(XY_spine) >= 1:
                        XY_spine = _snap_first_point(XY_spine, xy_all[-1][-1])
                    XY_spine[-1] = np.array(_latlon_to_xy_m(tlat, tlon, lat0, lon0), dtype=float)

                    xy_all.append(XY_spine)
                    ids_all   += [pid] * len(XY_spine)
                    kinds_all += ['arc'] * len(XY_spine)

                    if trace:
                        print(f"[arc3 {pid}] start={_tup((slat,slon))} via={_tup((vlat,vlon))} -> target={_tup((tlat,tlon))}")
                    anchors["prev"] = (tlat, tlon)
                    end_hdg = arc.heading_at(tlat, tlon)

                # --- bearings-defined arc (exact or CR fallback) ---
                elif "in_bearing_deg" in piece and "out_bearing_deg" in piece:
                    slat, slon = _resolve_point(piece.get("start", {"ref":"prev"}),
                                                anchors, lat0, lon0, heading0_deg)
                    tlat, tlon = _resolve_point(piece["target"],
                                                anchors, lat0, lon0, heading0_deg)
                    frame = str(piece.get("frame", "initial")).lower()
                    if frame is None:
                        s_frame = str(piece.get("start", {}).get("frame", "")).lower()
                        t_frame = str(piece.get("target", {}).get("frame", "")).lower()
                        frame = "local" if ("local" in (s_frame, t_frame)) else "initial"
                    frame = str(frame).lower()
                    base_hdg = float(anchors.get("prev_heading_deg", heading0_deg))
                    if frame == "local":
                        in_brg  = _wrap360(base_hdg + float(piece["in_bearing_deg"]))
                        out_brg = _wrap360(base_hdg + float(piece["out_bearing_deg"]))
                    else:
                        in_brg  = float(piece["in_bearing_deg"])
                        out_brg = float(piece["out_bearing_deg"])
                    kind_this = None
                    end_hdg = None
                    try:
                        arc = CircularArc.from_bearings(
                            slat, slon, in_brg,
                            tlat, tlon, out_brg,
                            turn=piece.get("turn", "auto"), seg_id=pid
                        )

                        total = (arc.end_angle - arc.start_angle) % (2*np.pi) if arc.turn_left else (arc.start_angle - arc.end_angle) % (2*np.pi)
                        ang = arc.start_angle + np.linspace(0.0, total, max(2, samples)) if arc.turn_left else \
                              arc.start_angle - np.linspace(0.0, total, max(2, samples))
                        x = arc.cx + arc.radius*np.cos(ang)
                        y = arc.cy + arc.radius*np.sin(ang)
                        ll = np.array([_xy_to_latlon_m(xx, yy, arc.lat0, arc.lon0) for xx, yy in zip(x, y)], dtype=float)
                        XY_spine = np.array([_latlon_to_xy_m(lat, lon, lat0, lon0) for lat, lon in ll], dtype=float)
                        kind_this = 'arc'
                        end_hdg = arc.heading_at(tlat, tlon)
                    except ValueError:
                        XY_spine = SpinePath._cr_fallback_between_bearings(
                            lat0, lon0,
                            slat, slon, in_brg,
                            tlat, tlon, out_brg,
                            samples=max(2, piece.get("samples", default_samples)),
                            dist_hint_m=piece.get("fb_cr_dist_m", defaults.get("fb_cr_dist_m")),
                            alpha=float(piece.get("fb_alpha", defaults.get("fb_alpha", 0.5)))
                        )
                        kind_this = 'cr'
                        dx, dy = XY_spine[-1] - XY_spine[-2]
                        end_hdg = _xy_to_bearing_deg(dx, dy)

                    if xy_all and len(XY_spine) >= 1:
                        XY_spine = _snap_first_point(XY_spine, xy_all[-1][-1])
                    XY_spine[-1] = np.array(_latlon_to_xy_m(tlat, tlon, lat0, lon0), dtype=float)

                    xy_all.append(XY_spine)
                    ids_all   += [pid] * len(XY_spine)
                    kinds_all += [kind_this] * len(XY_spine)

                    anchors["prev"] = (tlat, tlon)
                    anchors["prev_heading_deg"] = float(end_hdg if end_hdg is not None else heading0_deg)

                else:
                    raise ValueError("arc piece needs 'via' or bearings fields")
           
            elif ptype == "cr":
                pts_ll = piece.get("points")
                if pts_ll is None:
                    raise ValueError("cr piece requires 'points': [...]")
                ll = _resolve_points_list(pts_ll, anchors, lat0, lon0, heading0_deg)
                pts_xy = np.array([_latlon_to_xy_m(lat, lon, lat0, lon0) for lat,lon in ll], dtype=float)
                XY = cls._catmull_rom(pts_xy, samples_per_segment=max(2, samples), alpha=float(piece.get("alpha", 0.5)))
                if xy_all and len(XY) >= 1:
                    XY = _snap_first_point(XY, xy_all[-1][-1])
                XY[-1] = np.array(_latlon_to_xy_m(float(ll[-1,0]), float(ll[-1,1]), lat0, lon0), dtype=float)
                xy_all.append(XY); ids_all += [pid]*len(XY); kinds_all += ['cr']*len(XY)
                if trace:
                    print(f"[cr {pid}] pts[0]={_tup((float(ll[0,0]), float(ll[0,1])))} "
                          f"pts[-1]={_tup((float(ll[-1,0]), float(ll[-1,1])))}")
                anchors["prev"] = (float(ll[-1,0]), float(ll[-1,1]))
                dx, dy = XY[-1] - XY[-2]
                end_hdg = _xy_to_bearing_deg(dx, dy)
                if trace:
                    latp, lonp = anchors["prev"]
                    if np.isclose(_haversine_m(latp, lonp, lat0, lon0), 0.0, atol=1e-6):
                        print(f"!! WARNING: after piece {pid!r}, anchors['prev'] is still origin {_tup((lat0,lon0))}")

            else:
                raise ValueError(f"unknown piece type '{ptype}'")
            if end_hdg is None:
                end_hdg = heading0_deg
            anchors["prev_heading_deg"] = float(end_hdg)
        XY = np.vstack(xy_all)
        ids_arr = np.array(ids_all, dtype=object)

# distances between successive samples
        diffs = np.diff(XY, axis=0)
        same_xy = (np.hypot(diffs[:,0], diffs[:,1]) <= 1e-12)

# True where a new run starts (including very first point)
        boundaries = np.r_[True, ids_arr[1:] != ids_arr[:-1]]

        keep = np.ones(len(XY), dtype=bool)
# Keep if moved OR we're at the start of a run
        keep[1:] = (~same_xy) | boundaries[1:]
        keep[-1] = True

        XY = XY[keep]
        ids_all   = [i  for k,i  in zip(keep, ids_all)   if k]
        kinds_all = [kd for k,kd in zip(keep, kinds_all) if k]

        return cls(lat0, lon0, XY, ids_all, kinds_all)


    @staticmethod
    def _cr_fallback_between_bearings(lat0, lon0, start_lat, start_lon, in_brg_deg,
                                      target_lat, target_lon, out_brg_deg,
                                      samples=30, dist_hint_m=None, alpha=0.5):
        # anchor endpoints in ENU
        xA, yA = _latlon_to_xy_m(float(start_lat),  float(start_lon),  lat0, lon0)
        xB, yB = _latlon_to_xy_m(float(target_lat), float(target_lon), lat0, lon0)
        A = np.array([xA, yA], dtype=float)
        B = np.array([xB, yB], dtype=float)

        # choose fillet depth
        AB = B - A
        gap = float(np.hypot(AB[0], AB[1]))
        d = dist_hint_m if dist_hint_m is not None else 0.33 * gap
        d = float(np.clip(d, 2.0, 50.0))  # keep reasonable fillet sizes

        # tangent directions (0°=N, cw), ENU unit vectors
        uA = _bearing_to_unit_xy(float(in_brg_deg))          # leaving A
        uB = _bearing_to_unit_xy(float(out_brg_deg))         # leaving B
        A1 = A + d * uA
        B1 = B - d * uB                                      # approaching B

        # handle degenerate tiny gaps
        if gap < 1e-3:
            return np.linspace(A, B, max(2, samples))

        # Catmull–Rom through [A, A1, B1, B]
        ctrl = np.vstack([A, A1, B1, B])
        XY = SpinePath._catmull_rom(ctrl, samples_per_segment=max(2, samples), alpha=float(alpha))
        return XY

    # ---------- projection helpers ----------
    def _nearest_on_polyline(self, x, y):
        """
        Return (s_along, x_proj, y_proj, seg_idx, t_seg, signed_cte, heading_deg).
        """
        P = self.xy
        dP = np.diff(P, axis=0)
        seg_len = np.hypot(dP[:,0], dP[:,1]) + 1e-12
        # vector from segment starts to query point
        v = np.column_stack([x - P[:-1,0], y - P[:-1,1]])
        # projection param on each segment
        u = (v[:,0]*dP[:,0] + v[:,1]*dP[:,1]) / (seg_len**2)
        u_clamped = np.clip(u, 0.0, 1.0)
        proj = P[:-1] + dP * u_clamped[:,None]
        # pick closest projected point
        dist2 = np.sum((proj - np.array([x,y]))**2, axis=1)
        k = int(np.argmin(dist2))
        xq, yq = proj[k]
        # arclength at projection
        s_along = float(self.s[k] + u_clamped[k]*seg_len[k])
        # signed cross-track error (+right)
        tan = dP[k] / seg_len[k]
        err = np.array([x - xq, y - yq])
        z = tan[0]*err[1] - tan[1]*err[0]
        signed_cte = float(np.sign(z) * np.hypot(err[0], err[1]))
        heading_deg = float(_xy_to_bearing_deg(tan[0], tan[1]))
        return s_along, float(xq), float(yq), k, float(u_clamped[k]), signed_cte, heading_deg

    # ---------- Path interface ----------
    def along_fraction(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        s_along, *_ = self._nearest_on_polyline(x, y)
        if self.total <= _EPS: return 1.0
        return float(np.clip(s_along / self.total, 0.0, 1.0))

    def cross_track_error(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, _, _, signed_cte, _ = self._nearest_on_polyline(x, y)
        return float(signed_cte)

    def heading_at(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, _, _, _, hdg = self._nearest_on_polyline(x, y)
        return float(hdg)

    def distance_to_goal(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        s_along, *_ = self._nearest_on_polyline(x, y)
        return float(max(0.0, self.total - s_along))

    def length_m(self):
        return float(self.total)

    # ---------- “which piece am I on?” helpers ----------
    def active_piece_id(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, k, _, _, _ = self._nearest_on_polyline(x, y)
        return self.seg_ids_per_point[k]

    def active_piece_kind(self, lat, lon):
        x, y = _latlon_to_xy_m(lat, lon, self.lat0, self.lon0)
        _, _, _, k, _, _, _ = self._nearest_on_polyline(x, y)
        return self.seg_kinds_per_point[k]

    def list_pieces(self):
        # summarize contiguous runs of the same id
        ids = self.seg_ids_per_point
        kinds = self.seg_kinds_per_point
        out = []
        if not ids: return out
        start = 0
        for i in range(1, len(ids)+1):
            if i == len(ids) or ids[i] != ids[start]:
                piece_id = ids[start]
                kind = kinds[start]
                s0 = float(self.s[start])
                s1 = float(self.s[i]) if i < len(self.s) else float(self.s[-1])
                out.append({"id": piece_id, "kind": kind, "length_m": s1 - s0})
                start = i
        return out

# ---------------- composite ----------------

class MultiSegmentPath(Path):
    def __init__(self, segments, switch_margin_m: float = 1.0, end_advance_threshold: float = 0.995):
        assert len(segments) >= 1, "MultiSegmentPath needs at least one segment"

        # set segments first
        self.segs = list(segments)

        # --- ID handling ---
        used = set()
        for i, s in enumerate(self.segs):
            if getattr(s, 'id', None) is None:
                s.id = i                      # auto-id = index
            if s.id in used:
                raise ValueError(f"Duplicate segment id '{s.id}' at index {i}")
            used.add(s.id)

        # lengths & cumulative
        self.len = [s.length_m() for s in self.segs]
        self.cum = np.cumsum([0.0] + self.len)   # cum[i] = length up to seg i
        self.total = float(self.cum[-1]) if self.cum[-1] > 0 else 1.0

        # runtime state
        self.active_idx = 0
        self.switch_margin_m = float(switch_margin_m)
        self.end_advance_threshold = float(end_advance_threshold)

    def __repr__(self):
        return (f"<MultiSegmentPath n={len(self.segs)} "
                f"active={self.active_idx} len_m={self.total:.3f}>")

    def _update_active(self, lat, lon):
        i = self.active_idx
        n = len(self.segs)
        ti = self.segs[i].along_fraction(lat, lon)
        if ti >= self.end_advance_threshold and i < n - 1:
            self.active_idx = i + 1
            return self.active_idx

        if i < n - 1:
            e_cur  = np.abs(self.segs[i].cross_track_error(lat, lon))
            e_next = np.abs(self.segs[i+1].cross_track_error(lat, lon))
            if e_next + self.switch_margin_m < e_cur:
                self.active_idx = i + 1
        return self.active_idx

    def _progress_s(self, lat, lon):
        i = self._update_active(lat, lon)
        ti = float(self.segs[i].along_fraction(lat, lon))
        s  = float(self.cum[i] + ti * self.len[i])
        return max(0.0, min(self.total, s))

    # Path interface
    def along_fraction(self, lat, lon) -> float:
        return self._progress_s(lat, lon) / self.total

    def cross_track_error(self, lat, lon) -> float:
        i = self._update_active(lat, lon)
        return float(self.segs[i].cross_track_error(lat, lon))

    def heading_at(self, lat, lon) -> float:
        i = self._update_active(lat, lon)
        return float(self.segs[i].heading_at(lat, lon))

    def distance_to_goal(self, lat, lon) -> float:
        i = self._update_active(lat, lon)
        ti = float(self.segs[i].along_fraction(lat, lon))
        dist_on_i = (1.0 - ti) * self.len[i]
        trailing  = float(self.total - self.cum[i] - self.len[i])
        return float(dist_on_i + trailing)

    def length_m(self) -> float:
        return self.total

    def segment_kinds(self):
        """List of 'line'/'arc' for all segments in order."""
        return [getattr(s, 'kind', 'unknown') for s in self.segs]

    def active_segment_index(self, lat, lon) -> int:
        """Index of the currently active segment (after internal update)."""
        return int(self._update_active(lat, lon))

    def active_segment(self, lat, lon):
        """The active segment object."""
        return self.segs[self.active_segment_index(lat, lon)]

    def active_segment_kind(self, lat, lon) -> str:
        """'line' or 'arc' for the active segment."""
        return getattr(self.active_segment(lat, lon), 'kind', 'unknown')

    def active_progress_on_segment(self, lat, lon) -> float:
        """t in [0,1] along the active segment."""
        i = self._update_active(lat, lon)
        return float(self.segs[i].along_fraction(lat, lon))

    def segment_ids(self):
        """List of IDs (could be int or str) in order."""
        return [getattr(s, 'id', None) for s in self.segs]

    def active_segment_id(self, lat, lon):
        """ID of the active segment at (lat,lon)."""
        i = self._update_active(lat, lon)
        return getattr(self.segs[i], 'id', None)

    def find_segment_index_by_id(self, seg_id) -> int:
        for i, s in enumerate(self.segs):
            if getattr(s, 'id', None) == seg_id:
                return i
        raise KeyError(f"Segment id '{seg_id}' not found")

    def segment_by_id(self, seg_id):
        return self.segs[self.find_segment_index_by_id(seg_id)]

    def set_segment_id(self, idx_or_old_id, new_id):
        """Rename a segment. Accepts index or old id; enforces uniqueness."""
        if isinstance(idx_or_old_id, (int, np.integer)):
            i = int(idx_or_old_id)
        else:
            i = self.find_segment_index_by_id(idx_or_old_id)
        # uniqueness
        for k, s in enumerate(self.segs):
            if k != i and getattr(s, 'id', None) == new_id:
                raise ValueError(f"New id '{new_id}' already in use by segment {k}")
        self.segs[i].id = new_id
        return new_id

    def list_segments(self):
        out = []
        for i, s in enumerate(self.segs):
            item = {
                'index': i,
                'id': getattr(s, 'id', None),
                'kind': getattr(s, 'kind', 'unknown'),
                'length_m': float(s.length_m()),
            }
            if hasattr(s, 'latA') and hasattr(s, 'lonA'):
                item['start'] = {'lat': float(s.latA), 'lon': float(s.lonA)}
            if hasattr(s, 'latB') and hasattr(s, 'lonB'):
                item['end']   = {'lat': float(s.latB), 'lon': float(s.lonB)}
            if getattr(s, 'kind', None) == 'arc':
                if hasattr(s, 'r'):
                    item['radius_m'] = float(s.r)
                if hasattr(s, 'turn_left'):
                    item['turn'] = 'left' if s.turn_left else 'right'
                elif hasattr(s, 'turn'):
                    item['turn'] = str(s.turn)
            out.append(item)
        return out
# ---------------- factory ----------------

def _build_segment_from_dict(d: dict) -> Path:
    t = d['type'].strip().lower()
    seg_id = d.get('id', None)
    if t == 'line':
        return GeodesicLinePath(d['start_lat'], d['start_lon'], d['target_lat'], d['target_lon'], seg_id=seg_id)
    elif t == 'arc3':
        # use the older 3-point arc class if you insist, or the unified one:
        return CircularArc.from_three_points(d['start_lat'], d['start_lon'],
                                             d['via_lat'],   d['via_lon'],
                                             d['target_lat'],d['target_lon'],
                                             seg_id=seg_id)
    elif t == 'arc_brg':
        # safer: try exact arc, else CR fillet
        return arc_or_cr_from_bearings(d['start_lat'], d['start_lon'], d['in_bearing_deg'],
                                       d['target_lat'], d['target_lon'], d['out_bearing_deg'],
                                       turn=d.get('turn', 'auto'), seg_id=seg_id)
    else:
        raise ValueError(f"Unknown segment type '{t}'")

def arc_or_cr_from_bearings(latA, lonA, in_brg_deg, latB, lonB, out_brg_deg,
                            turn='auto', seg_id=None,
                            fb_samples=40, fb_dist_m=None, fb_alpha=0.5) -> Path:
    """
    Try exact arc; if infeasible, return a CR fillet Path (same API).
    """
    try:
        return CircularArc.from_bearings(latA, lonA, in_brg_deg, latB, lonB, out_brg_deg,
                                         turn=turn, seg_id=seg_id)
    except ValueError:
        return SplineFilletPath.from_bearings(latA, lonA, in_brg_deg,
                                              latB, lonB, out_brg_deg,
                                              lat0=latA, lon0=lonA,
                                              seg_id=seg_id,
                                              samples=fb_samples,
                                              dist_hint_m=fb_dist_m,
                                              alpha=fb_alpha)

def arc3_or_line(latA, lonA, latM, lonM, latB, lonB,
                 seg_id=None, fb_line_samples=10) -> Path:
    """
    Try exact 3-point arc; if points are colinear, fall back to a straight line.
    """
    try:
        return CircularArc.from_three_points(latA, lonA, latM, lonM, latB, lonB, seg_id=seg_id)
    except ValueError:
        return SplineFilletPath.from_line(latA, lonA, latB, lonB,
                                          lat0=latA, lon0=lonA,
                                          seg_id=seg_id,
                                          samples=fb_line_samples)

def build_path(nav_cfg) -> Path:
    """
    path_type = line | arc3 | arc_brg | sequence
    For sequence: set segments_json=<path/to/file.json> in [Nav]
    """
    path_type = nav_cfg.get('path_type', fallback='line').strip().lower()
    if path_type == 'sequence':
        json_path = nav_cfg.get('segments_json')
        if not json_path:
            raise ValueError("sequence path_type requires 'segments_json' in [Nav]")
        with open(json_path, 'r') as f:
            spec = json.load(f)
        segments = [_build_segment_from_dict(seg) for seg in spec['segments']]
        switch_margin = float(spec.get('switch_margin_m', 1.0))
        end_thresh    = float(spec.get('end_advance_threshold', 0.995))
        return MultiSegmentPath(segments, switch_margin_m=switch_margin, end_advance_threshold=end_thresh)

    # single-segment variants
    latA = float(nav_cfg['start_lat']); lonA = float(nav_cfg['start_lon'])
    latB = float(nav_cfg['target_lat']); lonB = float(nav_cfg['target_lon'])
    seg_id = nav_cfg.get('id', fallback=None)

    if path_type == 'line':
        return GeodesicLinePath(latA, lonA, latB, lonB, seg_id=seg_id)

    elif path_type == 'arc3':
        latM = float(nav_cfg['via_lat']); lonM = float(nav_cfg['via_lon'])
        return CircularArc.from_three_points(latA, lonA, latM, lonM, latB, lonB, seg_id=seg_id)

    elif path_type == 'arc_brg':
        in_brg  = float(nav_cfg['in_bearing_deg'])
        out_brg = float(nav_cfg['out_bearing_deg'])
        turn = nav_cfg.get('turn', fallback='auto').strip().lower()
        # safer:
        return arc_or_cr_from_bearings(latA, lonA, in_brg, latB, lonB, out_brg, turn=turn, seg_id=seg_id)

    else:
        raise ValueError(f"Unknown path_type '{path_type}'")

def debug_arc_bearings_feasibility(latA, lonA, in_brg_deg, latB, lonB, out_brg_deg):
    lat0, lon0 = float(latA), float(lonA)
    xA, yA = _latlon_to_xy_m(latA, lonA, lat0, lon0)
    xB, yB = _latlon_to_xy_m(latB, lonB, lat0, lon0)
    A = np.array([xA, yA], dtype=float); B = np.array([xB, yB], dtype=float)
    tA = _bearing_to_unit_xy(float(in_brg_deg))
    tB = _bearing_to_unit_xy(float(out_brg_deg))

    def side(name, nA, nB):
        C, lam, mu = _intersect_lines(A, nA, B, nB)
        ok = (C is not None) and (lam is not None) and (mu is not None) and (lam > 0) and (mu > 0)
        return dict(side=name, ok=bool(ok), lam=float(lam) if lam is not None else None,
                    mu=float(mu) if mu is not None else None,
                    C=(None if C is None else (float(C[0]), float(C[1]))))
    left  = side('left',  _left_normal(tA),  _left_normal(tB))
    right = side('right', _right_normal(tA), _right_normal(tB))
    return {"left": left, "right": right}

# Example:
# print(debug_arc_bearings_feasibility(37.0,-122.0, 0.0, 37.001,-122.0005, 90.0))


if __name__ == "__main__":
# 1) bearings arc feasible
    seg = arc_or_cr_from_bearings(
        37.0, -122.0,           # A
        0.0,                    # in_bearing_deg
        37.001, -122.0005,      # B
        90.0,                   # out_bearing_deg
        turn='auto',            # tries left/right; falls back if neither works
        seg_id='A1',
        fb_samples=48, fb_dist_m=12.0, fb_alpha=0.5
    )
    print(type(seg).__name__, getattr(seg, 'kind', None), seg.length_m())

# 2) bearings arc infeasible → fallback CR via factory
    p = arc_or_cr_from_bearings(37.0,-122.0, 0.0, 37.0002,-122.0006, 0.0, turn='left', seg_id='A2')
    print('kind', p.kind, 'len', p.length_m())

# 3) spine spec mixing line + arc_brg with auto-fallback
    spec = {
      "lat0": 37.0, "lon0": -122.0,
      "initial_bearing_deg": 0.0,   # 0° = forward = North
      "trace": True,                 # <-- turn on tracing

      "pieces": [
        # 10 m straight
        {"type":"line","id":"L1",
         "start":{"ref":"prev"},
         "target":{"ref":"prev","fwd_m":10.0},
         "samples":12},

        # U-turn right, radius 3 m:
        # chord for 180° arc = 2R → place target 2R to the 'right' of start
        {"type":"arc","id":"U1",
         "start":{"ref":"prev"},
         "target":{"ref":"prev","right_m": 2*3.0},
         "in_bearing_deg": 0.0, "out_bearing_deg": 180.0, "turn":"right",
         "samples":36, "fb_cr_dist_m": 3.0},

        # 10 m straight (still using initial bearing frame)
        {"type":"line","id":"L2",
         "start":{"ref":"prev"},
         "target":{"ref":"prev","fwd_m":10.0},
         "samples":12},

        # U-turn left, radius 3 m (mirror)
        {"type":"arc","id":"U2",
         "start":{"ref":"prev"},
         "target":{"ref":"prev","right_m": -2*3.0},
         "in_bearing_deg": 180.0, "out_bearing_deg": 0.0, "turn":"right",
         "samples":36, "fb_cr_dist_m": 3.0}
      ]
    }

    sp = SpinePath.from_spec(spec)
    print('spine total', sp.length_m(), 'pieces', sp.list_pieces())
    gj = path_to_geojson(sp, step_m=2.0)
    save_geojson(gj, "test_save.geojson")

