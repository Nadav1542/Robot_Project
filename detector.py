# Comments in English only
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

from ultralytics import YOLO

# candidate: (confidence, (x1,y1,x2,y2)) in pixels
Candidate = Tuple[float, Tuple[float, float, float, float]]

@dataclass
class DetectorConfig:
    min_conf: float = 0.35
    imgsz: int = 640
    label: str = "chair"   # target class name
    min_box_frac: float = 0.05  # min bbox height as fraction of frame height

class YoloDetector:
    def __init__(self, weights: str = "yolov8n.pt", cfg: Optional[DetectorConfig] = None):
        self.cfg = cfg or DetectorConfig()
        self.model = YOLO(weights)
        # Robust access to class names
        self.names = getattr(getattr(self.model, "model", self.model), "names", {})

    def detect(self, frame: np.ndarray) -> tuple[List[Candidate], Optional[any]]:
        """
        Run YOLO on BGR frame, return (candidates, raw_result).
        candidates are filtered by label, confidence and size.
        """
        res = self.model.predict(
            frame, imgsz=self.cfg.imgsz, conf=self.cfg.min_conf, verbose=False
        )[0]

        candidates: List[Candidate] = []
        h = frame.shape[0]
        if hasattr(res, "boxes") and res.boxes is not None:
            boxes = res.boxes.xyxy.cpu().numpy()
            clss  = res.boxes.cls.cpu().numpy().astype(int)
            confs = res.boxes.conf.cpu().numpy()

            for (x1, y1, x2, y2), cid, p in zip(boxes, clss, confs):
                name = self.names.get(int(cid), "")
                if name != self.cfg.label or float(p) < self.cfg.min_conf:
                    continue
                if (y2 - y1) < (self.cfg.min_box_frac * h):
                    continue
                candidates.append((float(p), (float(x1), float(y1), float(x2), float(y2))))
        return candidates, res

    @staticmethod
    def draw_all_boxes(frame: np.ndarray, res: any) -> None:
        """Optional: draw all detections (thin green boxes)."""
        if res is None or res.boxes is None:
            return
        for b in res.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = map(int, b)
            import cv2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
