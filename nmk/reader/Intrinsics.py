import yaml
from nmk.helper import yamlload

def getIntrinsics(drone_model):
    """
     Loads intrinsics from file and returns camera matrix and distortion coefficients. This function is used to load intrinsics from file and return camera matrix and distortion coefficients to be used in calibrate_drone.
     
     @param drone_model - String specifying the path to the drone model
     
     @return Camera matrix and distortion
    """
    drone_model = drone_model.lower()
    # Returns the path to the file containing the calibration data.
    if drone_model in ["p4", "phantom", "phantom4"]:
        path = "/mnt/00A03D3C0BCCF8D8/Image_data/Calibration/P4calib.yaml"
    elif drone_model in ["p4rtk", "phantom4rtk"]:
        path = "/mnt/00A03D3C0BCCF8D8/Image_data/Calibration/p4rtkcalib.yaml"
    else:
        raise IOError("Specified model path not defined : Path {}".format(drone_model))

    intrinsics = yamlload(path)
    mtx = intrinsics["camera_matrix"]
    dist_coefs = intrinsics["dist_coefs"]
    return mtx, dist_coefs


# Print out the phantom script.
if __name__ == "__main__":
    mtx, dist = getIntrinsics("phantom")
    print(mtx, dist)
