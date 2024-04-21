"""Utility functions that are standard to process """
import numpy as np
import cv2
import struct

def cv2_draw_polygon(img: np.ndarray,
                     points: list=[],
                     color: tuple=(22, 22, 22),
                     line_thickness: int=2) -> None:
    for i in range(len(points) - 1):
        cv2.line(img, points[i], points[i+1], color, thickness=line_thickness)
    cv2.line(img, points[len(points)-1], points[0], color, thickness=line_thickness)

def resize_img(img: np.ndarray, 
               camera_type: str, 
               img_w: int=128, 
               img_h: int=128, 
               offset_w: int=0, 
               offset_h: int=0,
               fx: float=None,
               fy: float=None) -> np.ndarray:
    if camera_type == "k4a":
        if fx is None:
            fx = 0.2
        if fy is None:
            fy = 0.2
        resized_img = cv2.resize(img, (0, 0), fx=fx, fy=fy)
        w = resized_img.shape[0]
        h = resized_img.shape[1]
    if camera_type == "rs":
        if fx is None:
            fx = 0.2
        if fy is None:
            fy = 0.3
        resized_img = cv2.resize(img, (0, 0), fx=fx, fy=fy)
        w = resized_img.shape[0]
        h = resized_img.shape[1]
    resized_img = resized_img[w//2-img_w//2:w//2+img_w//2, h//2-img_h//2:h//2+img_h//2, :]
    return resized_img


def cv2_undistort(color_img: np.ndarray,
                  intrinsics_matrix,
                  distortion):
    return cv2.undistort(color_img, intrinsics_matrix, distortion, None)


def depth_to_rgb(depth_img):
    # Normalize depth values between 0 and 1
    normalized_depth = cv2.normalize(depth_img, None, 0, 1, cv2.NORM_MINMAX, cv2.CV_32F)

    # Apply a colormap to the normalized depth image
    colormap = cv2.COLORMAP_JET
    depth_colormap = cv2.applyColorMap(np.uint8(normalized_depth * 255), colormap)
    return depth_colormap

def load_depth(depth_img_name):
    return cv2.imread(depth_img_name, cv2.IMREAD_UNCHANGED)

def save_depth(depth_img_name, depth_img):
    assert(depth_img_name.endswith(".tiff")), "You are not using tiff file for saving uint16 data. Things will be screwed."
    cv2.imwrite(depth_img_name, cv2.cvtColor(depth_img, cv2.CV_16U))

def load_depth_in_rgb(depth_img_name):
    rgb_img = cv2.imread(depth_img_name).astype(np.uint8)
    
    depth_img = np.zeros((rgb_img.shape[0], rgb_img.shape[1])).astype(np.uint16)
    depth_img = rgb_img[..., 1].astype(np.uint16) << 8 | rgb_img[..., 2].astype(np.uint16)

    return depth_img

def save_depth_in_rgb(depth_img_name, depth_img):
    """
    Saving depth image in the format of rgb images. The nice thing is that we can leverage the efficient PNG encoding to save almost 50% spaces compared to using tiff.
    """
    assert(depth_img.dtype == np.uint16)
    assert(depth_img_name.endswith(".png")), "You are not using lossless saving. Depth image will be messed up if you want to use rgb format."
    higher_bytes = depth_img >> 8
    lower_bytes = depth_img & 0xFF
    depth_rgb_img = np.zeros((depth_img.shape[0], depth_img.shape[1], 3)).astype(np.uint8)
    depth_rgb_img[..., 1] = higher_bytes.astype(np.uint8)
    depth_rgb_img[..., 2] = lower_bytes.astype(np.uint8)
    cv2.imwrite(depth_img_name, depth_rgb_img)

def preprocess_color(color_img, flip_channel=True):
    if flip_channel:
        return np.ascontiguousarray(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
    else:
        return np.ascontiguousarray(color_img)
    
def preprocess_depth(depth_img):
    return np.ascontiguousarray(depth_img)
    

def struct_encode_rgb(rgb_img):
    h, w, c = rgb_img.shape
    return struct.pack(">III", h, w, c) + rgb_img.tobytes()

def struct_decode_rgb(encoded_rgb):
    h, w, c = struct.unpack(">III", encoded_rgb[:12])
    return np.frombuffer(encoded_rgb[12:], dtype=np.uint8).reshape(h, w, c)

def struct_encode_depth(depth_img):
    h, w = depth_img.shape
    return struct.pack(">II", h, w) + depth_img.tobytes()

def struct_decode_depth(encoded_depth):
    h, w = struct.unpack(">II", encoded_depth[:8])
    return np.frombuffer(encoded_depth[8:], dtype=np.uint16).reshape(h, w)