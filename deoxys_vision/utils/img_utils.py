"""Utility functions that are standard to process """
import numpy as np
import cv2

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


def cv2_undistort(img: np.ndarray,
                  intrinsics_matrix,
                  distortion):
    return cv2.undistort(img_color, intrinsics_matrix, distortion, None)


def depth_to_rgb(depth_image):
    # Normalize depth values between 0 and 1
    normalized_depth = cv2.normalize(depth_image, None, 0, 1, cv2.NORM_MINMAX, cv2.CV_32F)

    # Apply a colormap to the normalized depth image
    colormap = cv2.COLORMAP_JET
    depth_colormap = cv2.applyColorMap(np.uint8(normalized_depth * 255), colormap)
    return depth_colormap

def load_depth(depth_image_name):
    return cv2.imread(depth_image_name, cv2.IMREAD_UNCHANGED)

def save_depth(depth_image_name, depth_image):
    cv2.imwrite(depth_image_name, cv2.cvtColor(depth_image, cv2.CV_16U))    
