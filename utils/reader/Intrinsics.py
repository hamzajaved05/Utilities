import yaml
from helper.utils import yamlload

def getIntrinsics(drone_model):
    drone_model = drone_model.lower()
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


if __name__ == "__main__":
    mtx, dist = getIntrinsics("phantom")
    print(mtx, dist)
