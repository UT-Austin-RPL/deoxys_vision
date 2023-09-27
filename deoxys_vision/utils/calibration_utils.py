import json

import numpy as np

from deoxys_vision import get_calibration_path

def load_default_extrinsics(camera_id,
                            camera_type,
                            calibration_method="tsai",
                            fmt="homogeneous"):
    default_extrinsic_json_file = os.path.join(get_calibration_path("results"),
                                               f"camera_{camera_id}_{camera_type}_{calibration_method}_extrinsics.json")
    
    assert(os.path.exists(default_extrinsic_json_file)), f"{default_extrinsic_json_file} needs to exist"

    with open(default_extrinsic_json_file, "r") as f:
        data = json.load(f)

    return get_extrinsics_data(data, fmt)

def get_extrinsics_data(data, fmt):
    if fmt == "homogeneous":
        homogeneous_matrix = np.eye(4)
        homogeneous_matrix[:3, :3] = data["rotation"]
        homogeneous_matrix[:3, 3] = data["translation"]
        return homogeneous_matrix
    else:
        raise NotImplementedError
