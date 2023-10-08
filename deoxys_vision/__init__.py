import os

ROOT_PATH = os.path.abspath(os.path.dirname(__file__))

calibration_config_folder = os.path.join(os.path.expanduser("~/"), ".deoxys_vision/calibration_configuration")
calibration_folder = os.path.join(os.path.expanduser("~/"), ".deoxys_vision/calibration")

os.makedirs(os.path.join(os.path.expanduser("~/"), calibration_config_folder), exist_ok=True)
os.makedirs(os.path.join(os.path.expanduser("~/"), calibration_folder), exist_ok=True)


def get_calibration_path(query):
    assert(query in ["results", "config"])
    if query == "results":
        return calibration_folder
    elif query == "config":
        return calibration_config_folder
