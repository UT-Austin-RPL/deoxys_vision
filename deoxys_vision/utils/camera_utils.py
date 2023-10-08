
from easydict import EasyDict

def assert_camera_ref_convention(camera_reference, 
                       available_camera_types=["rs", "k4a"]):
    """Check the camera reference convention. You need to check every camera separately.

    Args:
        camera_reference (str): _description_
        available_camera_types (list, optional): _description_. Defaults to ["rs", "k4a"].
    """
   
    assert(isinstance(camera_reference, str)), f"camera ref must be a string, got {type(camera_reference)} which is {camera_reference}-"
    camera_id = int(camera_reference.split("_")[-1])
    camera_type = camera_reference.split("_")[0]
    assert(camera_type in available_camera_types), f"camera_type must be one of {available_camera_types}, got {camera_type}"
    assert(camera_id >= 0), "camera id must be >= 0"

def get_camera_info(camera_reference):
    camera_id = int(camera_reference.split("_")[-1])
    camera_type = camera_reference.split("_")[0]
    camera_info = EasyDict({
        "camera_id": camera_id,
        "camera_type": camera_type,
        "camera_name": f"camera_{camera_reference}"
    })
    return camera_info