import numpy as np

BEARING_CONVENTION = 'CCW'   # set to 'CW' or 'CCW'
def haversine(lat1, lon1, lat2, lon2):
    """Calculate the distance between two lat/lon pairs in meters."""
    R = 6371000  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)

    a = (np.sin(delta_phi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda/2)**2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

    return R * c
def _wrap360(a):  # normalize degrees into [0,360)
    return float(a % 360.0)
def _cw_to_user(deg):
    """Convert a 'CW' internal bearing back to the selected convention."""
    deg = _wrap360(deg)
    return deg if BEARING_CONVENTION == 'CW' else _wrap360(360.0 - deg)
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

print(haversine(37.26026625,-121.8421298,37.2602656,-121.8420237))
print(_bearing_deg(37.26030078333333, -121.84412341666665, 37.2606779, -121.844123))
