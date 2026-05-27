# viz_path.py
from __future__ import annotations
import json, numpy as np
import matplotlib.pyplot as plt

# import ONLY from your path module
from path import SpinePath

R_EARTH = 6_371_000.0


def dbg_spine(sp: SpinePath, tol_m=1e-6):
    """Print run sizes and seam gaps (in ENU meters)."""
    ids = sp.seg_ids_per_point
    XY  = sp.xy
    lat0, lon0 = sp.lat0, sp.lon0

    # collect contiguous runs (id, start, end)
    runs = []
    start = 0
    for i in range(1, len(ids)+1):
        if i == len(ids) or ids[i] != ids[start]:
            runs.append((ids[start], start, i))
            start = i

    # summary
    print("runs (id, npts):", [(rid, end-start) for rid, start, end in runs])

    # per-run detail + seam checks
    for rix, (rid, a, b) in enumerate(runs):
        first, last = XY[a], XY[b-1]
        fll = _xy_to_latlon_m(first[0], first[1], lat0, lon0)
        lll = _xy_to_latlon_m(last[0],  last[1],  lat0, lon0)

        print(f"[{rix}:{rid}] ENU first=({first[0]:.6f},{first[1]:.6f}) "
              f"last=({last[0]:.6f},{last[1]:.6f})  "
              f"LL first=({fll[0]:.7f},{fll[1]:.7f}) last=({lll[0]:.7f},{lll[1]:.7f})")

        # seam to next run
        if rix < len(runs) - 1:
            next_first = XY[runs[rix+1][1]]
            seam = float(np.linalg.norm(next_first - last))
            flag = "OK" if seam <= tol_m else "GAP!"
            print(f"   seam→next |Δ| = {seam:.6g} m  [{flag}]")


def _xy_to_latlon_m(x, y, lat0, lon0):
    lat = lat0 + (y / R_EARTH) * (180.0 / np.pi)
    lon = lon0 + (x / (R_EARTH * np.cos(np.radians(lat0)))) * (180.0 / np.pi)
    return float(lat), float(lon)



def plot_spine(sp: SpinePath, step_m=2.0, annotate=True, title=None, mode="enu"):
    fig, ax = plt.subplots(figsize=(7, 7))
    lat0, lon0 = sp.lat0, sp.lon0
    ids = sp.seg_ids_per_point
    XY  = sp.xy

    start = 0
    while start < len(ids):
        end = start + 1
        while end < len(ids) and ids[end] == ids[start]:
            end += 1
        chunk = XY[start:end]
        if len(chunk) == 1:
            chunk = np.vstack([chunk[0], chunk[0] + 1e-6])

        if mode == "enu":
            ax.plot(chunk[:,0], chunk[:,1], linewidth=2)
            if annotate:
                mid = chunk[len(chunk)//2]
                ax.text(mid[0], mid[1], str(ids[start]))
        else:  # "geo" (old behavior)
            latlon = np.array([_xy_to_latlon_m(x, y, lat0, lon0) for x, y in chunk])
            ax.plot(latlon[:,1], latlon[:,0], linewidth=2)
            if annotate:
                mid = latlon[len(latlon)//2]
                ax.text(mid[1], mid[0], str(ids[start]))

        start = end

    if mode == "enu":
        ax.set_xlabel("x east (m)"); ax.set_ylabel("y north (m)")
    else:
        ax.set_xlabel("Longitude (deg)"); ax.set_ylabel("Latitude (deg)")

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle=":")
    ax.set_title(title or f"SpinePath ({mode})")
    plt.tight_layout()
    return fig, ax


def load_spec(path_or_obj):
    if isinstance(path_or_obj, str):
        with open(path_or_obj, "r") as f:
            return json.load(f)
    return path_or_obj

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python viz_path.py path_spec.json"); raise SystemExit(2)
    spec = load_spec(sys.argv[1])
    sp = SpinePath.from_spec(spec)
    # quick sanity: each run length
    runs = []
    ids = sp.seg_ids_per_point
    start = 0
    for i in range(1, len(ids)+1):
        if i == len(ids) or ids[i] != ids[start]:
            runs.append((ids[start], i-start))
            start = i
    print("runs (id, npts):", runs)
    dbg_spine(sp, tol_m=1e-6)
    fig, ax = plot_spine(sp, step_m=2.0, annotate=True, title=sys.argv[1], mode="enu")
    plt.show()
