# Comments in English only
import cv2
from typing import Tuple

# ROI types
RoiRect = Tuple[int, int, int, int]       # (x1, y1, x2, y2)
RoiGeom = Tuple[float, float, float]      # (cx, width, height)


def compute_roi(frame_shape: Tuple[int, int, int], roi_norm: Tuple[float, float, float, float]) -> Tuple[RoiRect, RoiGeom]:
    """
    Compute pixel coordinates and geometric info for the ROI.
    :param frame_shape: shape of frame (H, W, C)
    :param roi_norm: normalized coords (x_min, y_min, x_max, y_max)
    :return: (roi_rect, roi_geom)
             roi_rect = (rx1, ry1, rx2, ry2)
             roi_geom = (roi_cx, roi_w, roi_h)
    """
    h, w = frame_shape[:2]
    rx1 = int(roi_norm[0] * w)
    ry1 = int(roi_norm[1] * h)
    rx2 = int(roi_norm[2] * w)
    ry2 = int(roi_norm[3] * h)
    roi_w = float(rx2 - rx1)
    roi_h = float(ry2 - ry1)
    roi_cx = 0.5 * (rx1 + rx2)
    return (rx1, ry1, rx2, ry2), (roi_cx, roi_w, roi_h)


def draw_roi(frame, roi_rect: RoiRect, mode: str) -> None:
    """
    Draw the ROI on the given frame.
    - Green when in APPROACH or HOLD.
    - White otherwise.
    """
    rx1, ry1, rx2, ry2 = roi_rect
    color = (0, 255, 0) if mode in ("APPROACH", "HOLD") else (255, 255, 255)
    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), color, 2)
