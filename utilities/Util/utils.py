import numpy as np
import torch
import os
import json
import yaml

def recursive_decorator(func):
    def filter(obj, *args):
        if isinstance(obj, np.ndarray) or isinstance(obj, torch.Tensor):
            return func(obj, *args)
        elif isinstance(obj, list):
            return [func(element, *args) for element in obj]
        elif isinstance(obj, tuple):
            return (func(element, *args) for element in obj)
        elif isinstance(obj, dict):
            return {k:func(v, *args) for k,v in obj.items}
    return filter
    

def checkmkdir(input_path):
    abs_path = os.path.abspath(input_path)
    paths = abs_path.split("/")
    for i in range(2, len(paths) + 1):
        x = "/".join(paths[:i])
        if not os.path.isdir(x):
            os.mkdir(x)
    return True

def jsondump(path_, obj):
    with open(path_, 'w') as f:
        json.dump(obj, f)
        
def yamlload(path):
    if not os.path.isfile(path):
        raise IOError("Json path wrongly defined / Passed")
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def jsonload(path_):
    if os.path.isfile(path_):
        with open(path_, 'r') as f:
            return json.load(f)
    else:
        raise IOError("Json path wrongly defined / Passed")

def gamma_trans(img, gamma):
    gamma_table=[np.power(x/255.0,gamma)*255.0 for x in range(256)]
    gamma_table=np.round(np.array(gamma_table)).astype(np.uint8)
    return cv.LUT(img,gamma_table)

def str2bool(x):
    if x == "f" or x == "0" or x == "F" or x == "False" or x == "false":
        return False
    else:
        return True

if __name__ == "__main__":
    print(str2bool("1"))
    z = '/mnt/00A03D3C0BCCF8D8/Image_data_results'
    checkmkdir(z)
    