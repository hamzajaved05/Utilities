import numpy as np
import math

def metersdegLongitude(long: float):
    d2r = np.deg2rad(long)
    return ((111415.13 * np.cos(d2r)) - (94.55 * np.cos(3.0 * d2r)) + (0.12 * np.cos(5.0 * d2r)))

def metersdegLatitude(lat: float):
    d2r = np.deg2rad(lat)
    return (111132.09 - (566.05 * np.cos(2.0 * d2r)) + (1.20 * np.cos(4.0 * d2r)) - (0.002 * np.cos(6.0 * d2r)))

def CartesianMetersToGeo(Anchor, PointXY):
    lon = Anchor[1] + PointXY[0] / metersdegLongitude(Anchor[0])
    lat = Anchor[0] + PointXY[1] / metersdegLatitude(Anchor[0])
    return [lat, lon] if len(PointXY) == 2 else [lat, lon, PointXY[2]]


def GeoToCartesianMeters(latlon1, latlon2):
    x = (latlon2[1] - latlon1[1]) * metersdegLongitude(latlon1[0])
    y = (latlon2[0] - latlon1[0]) * metersdegLatitude(latlon1[0])
    return [x, y] if len(latlon1) == 2 else [x, y, latlon2[2] - latlon1[2]]

def CHtoWGS84(easting, northing, height = None):
    # constants
    a = 6378137
    f = 1 / 298.257223563  # WGS84
    e2 = 2 * f - f**2
    e = math.sqrt(e2)
    R = a * (1 - e2) / ((1 - e2 * math.sin(math.radians(46.95240555555556))**2)**(3 / 2))  # mean Earth radius at given latitude
    R_lon = R / math.cos(math.radians(46.95240555555556))

    # transforming to ellipsoidal coordinates (lat, lon)
    x_norm = (easting - 2600000) / 1000000
    y_norm = (northing - 1200000) / 1000000

    lon = 2.6779094 + 4.728982 * x_norm + 0.791484 * x_norm * y_norm + 0.1306 * x_norm * y_norm**2 - 0.0436 * x_norm**3
    lat = 16.9023892 + 3.238272 * y_norm - 0.270978 * x_norm**2 - 0.002528 * y_norm**2 - 0.0447 * x_norm**2 * y_norm - 0.0140 * y_norm**3
    lon = lon * 100 / 36
    lat = lat * 100 / 36

    # convert to degrees
    lon_deg = math.degrees(lon)
    lat_deg = math.degrees(lat)

    return [lat_deg, lon_deg] if height is None else [lat_deg, lon_deg, height]
