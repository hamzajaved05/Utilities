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
    scalar_dist = Haversine(ref_lon,ref_lat, cur_lon, cur_lat)

    angle = angleFromCoordinate(ref_lon, ref_lat, cur_lon, cur_lat)    
    angle = angle - 360 if angle > 180 else angle
    DisplAngle = (ref_gim % 360 - angle + 90) % 360

    DeltaYaw = cur_gim - ref_gim
    DeltaX = math.cos(DisplAngle / 180 * math.pi) * scalar_dist
    DeltaZ = math.sin(DisplAngle / 180 * math.pi) * scalar_dist
    rot = R.from_euler('y', [DeltaYaw], degrees=True).as_matrix().reshape([3,3])
    TransformationMatrix = np.concatenate([rot, [[DeltaX], [DeltaY], [DeltaZ]]], axis = 1)
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

if __name__ == "__main__":
    from Reader import imread

    if False:
        a = "/mnt/00A03D3C0BCCF8D8/Data/Technikum/images/DJI_0307.JPG"
        b = "/mnt/00A03D3C0BCCF8D8/Data/Technikum/images/DJI_0293.JPG"
        a_img, a_ex = imread(a)
        b_img, b_ex = imread(b)
        x = RelativeTransformExif(a_ex, b_ex)
        print(x)
    if True:
        import os
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        dir_ = "/mnt/00A03D3C0BCCF8D8/Data/Technikum/images"
        files = sorted([os.path.join(dir_, i) for i in os.listdir(dir_)])
        ref = files[0]
        ref_img, ref_ex = imread(ref)
        trans = []
        pos = np.array([0, 0, 1, 1])
        pose = []
        for cur in files[1:]:
            cur_img, cur_ex = imread(cur)
            t = RelativeTransformExif(ref_ex, cur_ex)
            pose.append(np.matmul(t, pos))
            trans.append(t)
        trans = np.array(trans)
        pose = np.array(pose)
        fig = plt.figure()    
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(trans[:, 0, -1], trans[:, 1, -1], trans[:, 2, -1])
        ax.plot(pose[:, 0], pose[:, 1], pose[:, 2])
        plt.show()