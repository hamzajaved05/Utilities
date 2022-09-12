import cv2 as cv
# from libxmp.utils import file_to_dict
import PIL.Image
import PIL.ExifTags
import time 
import numpy as np

# def get_xmp(path, dict_ = {}):
#     xmp = file_to_dict(path)
#     if len(xmp) == 0: return dict_
#     for i in xmp["http://www.dji.com/drone-dji/1.0/"]:
#         try:
#             dict_[i[0][10:]] = float(i[1])
#         except:
#             pass
#     return dict_

def get_exif(img_path, meta = {}):
    start = time.perf_counter()
    img = cv.imread(img_path)
    img_pil = PIL.Image.open(img_path)
    for k, v in img_pil._getexif().items():
        if k in PIL.ExifTags.TAGS:
            meta[PIL.ExifTags.TAGS[k]]= v
    x = meta["GPSInfo"][2]
    y = meta["GPSInfo"][4]
    h = meta["GPSInfo"][6]
    meta["Latitude"] = float(x[2] / 3600 + x[1] / 60 + x[0])
    meta["Longitude"] = float(y[2] / 3600 + y[1] / 60 + y[0])
    meta["RelativeAltitude"] = h
    return img, meta 

def imread(path, undistort = False, drone_model = None, get_intrinsics = None):
    img, meta = get_exif(path, {})
    # meta = get_xmp(path, meta)
    if undistort:
        assert drone_model is not None
        from Reader.Intrinsics import getIntrinsics
        mtx, dist = getIntrinsics(drone_model)
        # new_mtx = np.array(mtx)
        img_und = cv.undistort(img, np.array(mtx), np.array(dist), newCameraMatrix=None)
        
    if get_intrinsics:
        assert drone_model is not None
        from Reader.Intrinsics import getIntrinsics
        mtx, dist = getIntrinsics(drone_model)
        meta["K"] = mtx
        meta["dist"] = dist[0]
    
    return img, meta

if __name__ == "__main__":
    path = '/mnt/00A03D3C0BCCF8D8/Image_data/Self/Technikum/DJI_0278.JPG'
    img, meta = imread(path, get_intrinsics= True, undistort = True, drone_model = "phantom")
    print(meta)