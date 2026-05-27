
import json
# adjust this import to wherever your path module lives
from path import SpinePath, path_to_geojson, save_geojson

def main():
    with open("test.json", "r") as f:
        spec = json.load(f)

    sp = SpinePath.from_spec(spec)

    # quick summary
    print(sp)                      # __repr__
    for piece in sp.list_pieces(): # per-piece summary
        print(piece)

    # sample a couple queries
    lat0, lon0 = spec["lat0"], spec["lon0"]
    print("t@origin:", sp.along_fraction(lat0, lon0))
    print("hdg@origin:", sp.heading_at(lat0, lon0))
    print("total length (m):", sp.length_m())

    # export for quick map viewing (QGIS/kepler.gl/Mapbox)
    gj = path_to_geojson(sp, step_m=2.0)
    save_geojson(gj, "example_path.geojson")
    print("Wrote example_path.geojson")

if __name__ == "__main__":
    main()
