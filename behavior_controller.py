# Comments in English only
from dataclasses import dataclass
from typing import Any, Dict, Tuple, List, Optional

# Types
Candidate = Tuple[float, Tuple[float, float, float, float]]  # (conf, (x1,y1,x2,y2))


@dataclass
class BehaviorConfig:
    # Gains / tolerances
    K_WZ: float = 1.2
    K_VX_FWD: float = 0.8
    K_VX_BACK: float = 0.4
    CENTER_TOL: float = 0.10
    SIZE_TOL: float = 0.08

    # Limits (used only for yaw clamp here; speed clamp handled in FollowController)
    MAX_WZ: float = 0.96

    # Timing
    HOLD_SECONDS: float = 3.0
    COOLDOWN_SECONDS: float = 8.0


class BehaviorController:
    """
    Encapsulates the FOLLOW/APPROACH/HOLD state machine.
    - Mutates the shared `behavior` dict (vx, wz, mode, cooldown, target_box, roi_px).
    - Uses an external TargetLock instance for target association.
    """

    def __init__(
        self,
        behavior: Dict[str, Any],
        lock: Any,  # TargetLock
        cfg: Optional[BehaviorConfig] = None,
    ):
        self.behavior = behavior
        self.lock = lock
        self.cfg = cfg or BehaviorConfig()
        self._hold_until: float = 0.0
        self._last_announce: float = 0.0

    def step(
        self,
        roi_px: Tuple[int, int, int, int],
        roi_geometry: Tuple[float, float, float],  # (roi_cx, roi_w, roi_h)
        candidates: List[Candidate],
        now: float,
    ) -> Optional[Tuple[int, int, int, int]]:
        """
        Run one state-machine step.
        Returns a box to highlight (int x1,y1,x2,y2) if available, else None.
        """
        rx1, ry1, rx2, ry2 = roi_px
        roi_cx, roi_w, roi_h = roi_geometry

        # Keep ROI for awareness (read by motion thread if needed)
        self.behavior["roi_px"] = roi_px

        mode = self.behavior.get("mode", "FOLLOW")

        if mode == "FOLLOW":
            return self._step_follow(candidates, now, (rx1, ry1, rx2, ry2))

        if mode == "APPROACH":
            return self._step_approach(candidates, now, roi_cx, roi_w, roi_h)

        if mode == "HOLD":
            return self._step_hold(now)

        # Fallback
        self.behavior["mode"] = "FOLLOW"
        return None

    # -------------------- Internals --------------------
    def _step_follow(
        self,
        candidates: List[Candidate],
        now: float,
        roi_rect: Tuple[int, int, int, int],
    ) -> Optional[Tuple[int, int, int, int]]:
        # Ready to acquire only if cooldown passed
        if now >= float(self.behavior.get("cooldown_until", 0.0)):
            if not self.lock.active and candidates:
                if self.lock.acquire(candidates, roi_rect=roi_rect):
                    self.behavior["mode"] = "APPROACH"
                    self.behavior["target_box"] = self.lock.box
                    self.behavior["vx"] = 0.0
                    self.behavior["wz"] = 0.0
        return None

    def _step_approach(
        self,
        candidates: List[Candidate],
        now: float,
        roi_cx: float,
        roi_w: float,
        roi_h: float,
    ) -> Optional[Tuple[int, int, int, int]]:
        # Maintain/update lock; if lost -> back to FOLLOW
        if self.lock.active:
            self.lock.update(candidates)

        if not self.lock.active or self.lock.box is None:
            self.behavior["mode"] = "FOLLOW"
            self.behavior["target_box"] = None
            return None

        # Compute control towards the locked target
        x1, y1, x2, y2 = self.lock.box
        cx = 0.5 * (x1 + x2)
        bh = float(y2 - y1)
        ex = (cx - roi_cx) / max(roi_w, 1.0)  # left<0, right>0
        size_ratio = bh / max(roi_h, 1.0)
        ey = 1.0 - size_ratio                 # >0: need to get closer

        # yaw control
        if abs(ex) < self.cfg.CENTER_TOL:
            wz_t = 0.0
        else:
            wz_t = -self.cfg.K_WZ * ex
            # clamp
            wz_t = max(-self.cfg.MAX_WZ, min(self.cfg.MAX_WZ, wz_t))

        # forward/back control
        if abs(ey) < self.cfg.SIZE_TOL:
            # good alignment and distance -> HOLD
            self.behavior["vx"] = 0.0
            self.behavior["wz"] = 0.0
            self._hold_until = now + self.cfg.HOLD_SECONDS
            self.behavior["mode"] = "HOLD"
            print("[APPROACH] Target reached → HOLD")
        elif ey > 0.0:
            self.behavior["vx"] = self.cfg.K_VX_FWD * min(ey, 1.0)
            self.behavior["wz"] = wz_t
        else:
            self.behavior["vx"] = -self.cfg.K_VX_BACK * min(-ey, 1.0)
            self.behavior["wz"] = wz_t

        self.behavior["target_box"] = self.lock.box

        # Return box to draw (highlight)
        return (int(x1), int(y1), int(x2), int(y2))

    def _step_hold(self, now: float) -> Optional[Tuple[int, int, int, int]]:
        # Stand still
        self.behavior["vx"] = 0.0
        self.behavior["wz"] = 0.0

        if now - self._last_announce >= 1.0:
            print("Found chair — holding position…")
            self._last_announce = now

        if now >= self._hold_until:
            print("Found chair — returning to follow.")
            self.behavior["mode"] = "FOLLOW"
            self.behavior["cooldown_until"] = now + self.cfg.COOLDOWN_SECONDS
            self.lock.reset()
            self.behavior["target_box"] = None

        return None
