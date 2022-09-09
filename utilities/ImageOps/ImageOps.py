import cv2 as cv

def HistogramEqualize(img, size = 1.25):
    img = cv.cvtColor(img, cv.COLOR_BGR2YUV)
    tile_size = int(img.shape[0] * size / 100)
    clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(tile_size,tile_size))
    img[:,:,0] = clahe.apply(img[:,:,0])
    img = cv.cvtColor(img, cv.COLOR_YUV2BGR)
    return img

def AdjustWhiteBalance(img):
    R, G, B = cv.split(img)
    RMax = maxWhiteBalance(R)
    GMax = maxWhiteBalance(G)
    BMax = maxWhiteBalance(B)
    RN = cv.normalize(np.clip(R, 0, RMax), dst = None, alpha = 0, beta = 255, norm_type = cv.NORM_MINMAX)
    GN = cv.normalize(np.clip(G, 0, RMax), dst = None, alpha = 0, beta = 255, norm_type = cv.NORM_MINMAX)
    BN = cv.normalize(np.clip(B, 0, RMax), dst = None, alpha = 0, beta = 255, norm_type = cv.NORM_MINMAX)
    img = cv.merge([np.round(RN).astype("uint8"), np.round(GN).astype("uint8"), np.round(BN).astype("uint8")])
    return img

def maxWhiteBalance(channel):
    val, edges = np.histogram(channel, bins = 255, normed = True )
    index =  255 - (np.sum(np.cumsum(val) > (1-0.0005)))
    return edges[index] + 1

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    img = cv.imread("/mnt/00A03D3C0BCCF8D8/Codebase/Mosaicing/downscaled_result.jpg")[:, :, ::-1]
    balanced = AdjustWhiteBalance(img)
    plt.imshow(np.hstack([img, balanced]))
    plt.show()