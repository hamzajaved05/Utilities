import cv2 as cv
from libxmp.utils import file_to_dict
import PIL.Image
import PIL.ExifTags
import time 
from nmk.reader.Intrinsics import getIntrinsics
import numpy as np

def get_xmp(path, dict_ = {}):
    """
     Get xmp data from file and return as dict. If file doesn't exist return empty dict
     
     @param path - path to xmp file to read
     @param dict_ - dict to fill with data from xmp file
     
     @return dict_ with xmp data from file or empty dict if nothing found in xmp file or no
    """
    xmp = file_to_dict(path)
    # Return dict_ if len xmp 0.
    if len(xmp) == 0: return dict_
    # Convert from xmp xmp xmp dji. com drone dji 1. 0 to float
    for i in xmp["http://www.dji.com/drone-dji/1.0/"]:
        try:
            dict_[i[0][10:]] = float(i[1])
        except:
            pass
    return dict_

def get_exif(img_path, meta = {}):
    """
     Get EXIF data from image. This is a wrapper around PIL's _getexif function to allow us to pass metadata to the image as part of the metadata retrieval process
     
     @param img_path - path to image to get EXIF data from
     @param meta - dictionary of metadata to add to the image.
     
     @return tuple of image and exif metadata ( dict of key / value pairs ). The keys are strings and the values are lists
    """
    start = time.perf_counter()
    img = cv.imread(img_path)
    img_pil = PIL.Image.open(img_path)
    # Add tags to the meta dict.
    for k, v in img_pil._getexif().items():
        # Add tags to the meta data.
        if k in PIL.ExifTags.TAGS:
            meta[PIL.ExifTags.TAGS[k]]= v
    x = meta["GPSInfo"][2]
    y = meta["GPSInfo"][4]
    h = meta["GPSInfo"][6]
    meta["Latitude"] = float(x[2] / 3600 + x[1] / 60 + x[0])
    meta["Longitude"] = float(y[2] / 3600 + y[1] / 60 + y[0])
    meta["AbsoluteAltitude"] = h
    return img, meta 

def imread(path, undistort = False, drone_model = None, get_intrinsics = None):
    """
     Read image from path and return it as numpy array. If undistort is True image will be undistorted before returning it
     
     @param path - path to image file to read
     @param undistort - if True image will be undistorted
     @param drone_model - if specified will be used to get intrinsics
     @param get_intrinsics - if True will be used to get intrinsic parameters
     
     @return tuple of image and meta ( dict of image metadata ) note :: This function is a wrapper for cv. get_exif and get_xmp
    """
    img, meta = get_exif(path, {})
    meta = get_xmp(path, meta)
    # Undistort the image by undistorting the drone_model.
    if undistort:
        assert drone_model is not None
        mtx, dist = getIntrinsics(drone_model)
        new_mtx = np.array(mtx)
        img_und = cv.undistort(img, np.array(mtx), np.array(dist), newCameraMatrix=None)
        
    # Get the intrinsics of the drone model.
    if get_intrinsics:
        assert drone_model is not None
        mtx, dist = getIntrinsics(drone_model)
        meta["K"] = mtx
        meta["dist"] = dist[0]
    
    return img, meta

# This function is called from the main module.
if __name__ == "__main__":
    path = '/mnt/00A03D3C0BCCF8D8/Image_data/Self/Technikum/DJI_0278.JPG'
    img, meta = imread(path, get_intrinsics= True, undistort = True, drone_model = "phantom")
    print(meta)