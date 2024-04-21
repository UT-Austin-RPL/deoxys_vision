import os
import json

import numpy as np

from deoxys_vision import get_calibration_path

def load_default_extrinsics(camera_id,
                            camera_type,
                            calibration_method="tsai",
                            fmt="dict"):
    default_extrinsic_json_file = os.path.join(get_calibration_path("results"),
                                               f"camera_{camera_type}_{camera_id}_{calibration_method}_extrinsics.json")
    
    assert(os.path.exists(default_extrinsic_json_file)), f"{default_extrinsic_json_file} needs to exist"

    with open(default_extrinsic_json_file, "r") as f:
        data = json.load(f)
    return get_extrinsics_data(data, fmt)

def load_default_intrinsics(camera_id,
                            camera_type,
                            image_type="color",                            
                            fmt="dict",
):
    default_intrinsic_json_file = os.path.join(get_calibration_path("results"),
                                               f"camera_{camera_type}_{camera_id}_intrinsics.json")
    
    assert(os.path.exists(default_intrinsic_json_file)), f"{default_intrinsic_json_file} needs to exist"

    with open(default_intrinsic_json_file, "r") as f:
        data = json.load(f)
    return get_intrinsics_data(data[image_type], fmt)


def write_calibrated_extrinsics(data,
                                save_folder,
                                camera_id,
                                camera_type,
                                calibration_method="tsai",
):

    for key in ["translation", "rotation", "calibration_type"]:
        assert(key in data), f"{key} needs to be there for calibration"
    default_extrinsic_json_file = os.path.join(save_folder,
                                               f"camera_{camera_type}_{camera_id}_{calibration_method}_extrinsics.json")
    
    with open(default_extrinsic_json_file, "w") as f:
        json.dump(data, f)
    assert(os.path.exists(default_extrinsic_json_file)), f"{default_extrinsic_json_file} needs to exist"
    return default_extrinsic_json_file


# def load_default_extrinsics(camera_id,
#                             camera_type,
#                             calibration_method="tsai",
#                             fmt="homogeneous"):
#     default_extrinsic_json_file = os.path.join(get_calibration_path("results"),
#                                                f"camera_{camera_type}_{camera_id}_{calibration_method}_extrinsics.json")
    
#     assert(os.path.exists(default_extrinsic_json_file)), f"{default_extrinsic_json_file} needs to exist"

#     with open(default_extrinsic_json_file, "r") as f:
#         data = json.load(f)
#     return get_extrinsics_data(data, fmt)

def get_extrinsics_data(data, fmt):
    if fmt == "matrix":
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = data["rotation"]
        if type(data["translation"]) is list:
            data["translation"] = np.array(data["translation"])
        homogeneous_matrix[:3, 3:] = data["translation"]
        return homogeneous_matrix
    elif fmt == "dict":
        return data
    else:
        raise NotImplementedError

def get_intrinsics_data(data, fmt):
    if fmt == "matrix":
        intrinsic_matrix = np.eye(3)
        intrinsic_matrix[0][0] = data["fx"]
        intrinsic_matrix[1][1] = data["fy"]

        intrinsic_matrix[0][2] = data["cx"]
        intrinsic_matrix[1][2] = data["cy"]
        return intrinsic_matrix

    elif fmt == "dict":
        return data
    else:
        raise NotImplementedError

