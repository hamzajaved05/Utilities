from scipy.spatial.transform import Rotation as R
import math
import numpy as np

def get_depth(mkpts0, mkpts1, index, transform):
    import sympy as sym

    Pixel1 = np.concatenate([mkpts1, np.ones([len(mkpts1), 1])], axis = 1)[index]

    K = np.array([[298.4, 0, 320],[0, 298.4, 180], [0, 0, 1]])
    K = np.array([[170.18, 0, 320],[0, 223.8, 180], [0, 0, 1]])

    InCamera1 = np.ones(4)
    InCamera1[:3] = np.matmul(np.linalg.pinv(K), Pixel1)
    d = sym.symbols("d")

    InCamera1 = sym.Matrix(InCamera1)
    InCamera1[2] = "d"

    InWorld = np.matmul(transform, InCamera1)
    InCamera0 = np.matmul(K, InWorld)
    Pixel0Pred = InCamera0 / InCamera0[2]
    depth = sym.solvers.solve(Pixel0Pred[0] - mkpts0[index][0])[d]
    return depth

def RelativeTransform(ref_lon, ref_lat, ref_z, ref_gim, cur_lon, cur_lat, cur_z, cur_gim):
    """Get the relative transform from two drone positions

    Args:
        ref_lon (float): Longitude of the reference image
        ref_lat (float): Latitude of the reference image 
        ref_z (float): Altitude of reference image
        ref_gim (float): Gimbal Angle of the referencen image in degrees
        cur_lon (float): Longitude of the current image
        cur_lat (float): Latitude of the current image 
        cur_z (float): Altitude of the current image 
        cur_gim (float): Gimbal angle of the current image in degrees 

    Returns:
        np.ndarray: Tranformation matrix of the current frame wrt to the previous frame 
    """
    DeltaY = -(cur_z - ref_z)
    scalar_dist = Haversine(ref_lon, ref_lat, cur_lon, cur_lat)

    angle = angleFromCoordinate(ref_lon, ref_lat, cur_lon, cur_lat)    
    angle = angle - 360 if angle > 180 else angle
    DisplAngle = (ref_gim % 360 - angle + 90) % 360

    DeltaYaw = cur_gim - ref_gim
    DeltaX = math.cos(DisplAngle / 180 * math.pi) * scalar_dist
    DeltaZ = math.sin(DisplAngle / 180 * math.pi) * scalar_dist
    rot = R.from_euler('y', [DeltaYaw], degrees=True).as_matrix()[0]
    TransformationMatrix = np.concatenate([rot, [[DeltaX], [DeltaY], [DeltaZ]]], axis = 1)
    TransformationMatrix = np.concatenate([TransformationMatrix, [[0, 0, 0, 1]]], axis = 0)

    return TransformationMatrix

def HomogeniseRt(R, t):
    TransformationMatrix = np.concatenate([R, [[t[0]], [t[1]], [t[2]]]], axis = 1)
    TransformationMatrix = np.concatenate([TransformationMatrix, [[0, 0, 0, 1]]], axis = 0)
    return TransformationMatrix

def Haversine(lon1, lat1, lon2, lat2):
    """Calculate the haversine distanceo of two long and latitude position on the glob

    Args:
        lon1 (float): Longitude of Position 1
        lat1 (float): Latitudeo of Position 1 
        lon2 (float): Longitude of Position 2 
        lat2 (float): Latitude of Position 2 

    Returns:
        float: Distance between these two points in meters
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(np.sqrt(a)) 
    r = 6378.137 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return c * r * 1000 # For meters

def metersdegLongitude(long: float):
    d2r = np.deg2rad(long)
    return ((111415.13 * np.cos(d2r)) - (94.55 * np.cos(3.0 * d2r)) + (0.12 * np.cos(5.0 * d2r)))

def metersdegLatitude(lat: float):
    d2r = np.deg2rad(lat)
    return (111132.09 - (566.05 * np.cos(2.0 * d2r)) + (1.20 * np.cos(4.0 * d2r)) - (0.002 * np.cos(6.0 * d2r)));

def geoToMeters(latlon1, latlon2):
    x = (latlon2[1] - latlon1[1]) * metersdegLongitude(latlon1[0])
    y = (latlon2[0] - latlon1[0]) * metersdegLatitude(latlon1[0])
    return x, y

def exiftometers(exif1, exif2):
    latlon1 = [exif1["Latitude"], exif1["Longitude"]]
    latlon2 = [exif2["Latitude"], exif2["Longitude"]]
    return geoToMeters(latlon1, latlon2)

def angleFromCoordinate(long1, lat1, long2, lat2):
    """Calculate the angle between the position on the map in North East down convention

    Args:
        long1 (float): Longitude of postion 1 
        lat1 (float): Latitude of position 1
        long2 (float): Longitude of position 2
        lat2 (float): Latitude of position 2

    Returns:
        float: Angle in degrees between two locations in North East Down system
    """
    long1, lat1, long2, lat2 = map(math.radians, [long1, lat1, long2, lat2])

    dLon = (long2 - long1)

    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

    brng = math.atan2(y, x)

    brng = math.degrees(brng)
    brng = (brng + 360) % 360
    return brng

def RelativeTransformExif(ref_ex, cur_ex):
    return RelativeTransform(ref_ex["Longitude"], ref_ex["Latitude"], ref_ex["RelativeAltitude"], ref_ex["GimbalYawDegree"], cur_ex["Longitude"], cur_ex["Latitude"], cur_ex["RelativeAltitude"], cur_ex["GimbalYawDegree"])

def getUsefulData(exif):
    return [exif["Latitude"], exif["Longitude"], exif["RelativeAltitude"], exif["GimbalRollDegree"], exif["GimbalPitchDegree"], exif["GimbalYawDegree"]]

def RelativeTransformNew(anchor, query):
    anchor_data = getUsefulData(anchor)
    query_data = getUsefulData(query)

    p_x, p_y = exiftometers(anchor, query)
    p_z = query_data[2] - anchor_data[2]
    R_A = getRotationMatrixforFrame(anchor_data[3:])
    R_B = getRotationMatrixforFrame(query_data[3:])
    t_Cart = np.asarray([p_x, p_y, p_z])
    t_A = R_A.T.dot(t_Cart)
    R_BA = R_A.T.dot(R_B)

    # R_21 = getRelativeRotation(np.array(anchor_data[3:]), np.array(query_data[3:]))
    T_BA = HomogeniseRt(R_BA, t_A)

    # T_final = RotateCartesian2Camera(T_21)

    return T_BA

def RotateCartesian2Camera(trans: np.ndarray):
    rot = R.from_euler("xyz", [-90, 0, 0], degrees = True).as_matrix()
    if len(trans) == 4:
        factor = HomogeniseRt(rot, [0, 0, 0])
    else:
        factor = rot

    return trans.dot(factor)
    

def getRotationMatrixforFrame(rpy, NED = True):
    r, p, y = rpy[0], rpy[1], rpy[2]
    if NED:
        y = 90 - y
    y -= 90
    rpy = [r, p, y]
    Rot = R.from_euler("xyz", rpy, degrees = True).as_matrix()
    return RotateCartesian2Camera(Rot)

def getRelativeRotation(rpy1, rpy2, NED = True):
    rpyDelta = rpy2 - rpy1
    if NED:
        rpyDelta = -rpyDelta
    return R.from_euler("xyz", rpyDelta, degrees = True).as_matrix()

if __name__ == "__main__":
    from nmk.reader import imread

    if True:
        a = "/mnt/00A03D3C0BCCF8D8/Image_data/Self/Effretikon/spiral1/DJI_0201.JPG"
        b = "/mnt/00A03D3C0BCCF8D8/Image_data/Self/Effretikon/spiral1/DJI_0202.JPG"
        a_img, a_ex = imread(a)
        b_img, b_ex = imread(b)
        y = RelativeTransformExif(a_ex, b_ex)

        x = RelativeTransformNew(a_ex, b_ex)
        # p_x, p_y = exiftometers(a_ex, b_ex)
        print(x)