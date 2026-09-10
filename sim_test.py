"""
1.py — IARC line-follower demo

Architecture (swappable halves):
  Controller : FrameSimulator  (swap for real robot driver)
  Logic      : LineFollower    (takes frame → returns vx, vy)

Covers the PS requirements minus dead-end / barrier detection:
  • follows black line via look-ahead centroid + heading tracking
  • handles T-junctions and curves by finding the furthest forward
    extent of the line and steering toward it
"""

import cv2
import numpy as np
import sys
sys.path.insert(0, "/home/palash/iarc2026/experiments/isolation_testing/simulator")
from frame_simulator import FrameSimulator
from time import sleep

###r
# ---------------- Motor 1 Pins ----------------
PWM_PIN_MOT1 = 13
IN1_PIN_MOT1 = 5
IN2_PIN_MOT1 = 6


##l
# ---------------- Motor 2 Pins ----------------
PWM_PIN_MOT2 = 12
IN1_PIN_MOT2 = 23
IN2_PIN_MOT2 = 24



# ── configuration ─────────────────────────────────────────────────────
MAP_PATH        = "map.png"
FRAME_SIZE      = 50
FPS             = 2
BASE_SPEED      = 100
KP_YAW          = 0.05
START_X         = 101
START_Y         = 670
START_THETA     = -np.pi / 2  # Facing North (Top)


# Motor 1
def control_bot(vx, vy, omega):
    # Simulation only — no physical GPIO motors
    pass


class LineFollower:
    def __init__(self):
        self.Kp = KP_YAW
        self.speed = BASE_SPEED
        self.last_target = None  # To smooth or visualize

    def process(self, frame: np.ndarray) -> tuple[float, float, float]:
        """
        Process the frame and return (vx, vy, omega) for the robot.
        vx: forward velocity
        vy: strafe velocity (usually 0)
        omega: yaw rate
        """
        # ── Image processing pipeline from core.py ──
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        otsu_val, _ = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, mask = cv2.threshold(blurred, otsu_val, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_not(mask) # Black line on white bg -> White mask

        # Morphological close/open
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)), iterations=3)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)), iterations=1)

        # Smooth mask edges
        mask = cv2.GaussianBlur(mask, (51, 51), 0)
        _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

        # ── Analysis ──
        height, width = mask.shape
        # Fixed point: Center width, 35% height (from top)
        fixed_pt = (width // 2, int(height * 0.35))
        
        # Define ROIs (Top row)
        # Priorities: Left > Blue(Middle) > Right
        rois = [
            ("Left",   0, width // 2, 0, height // 4, (0, 255, 0)),
            ("Blue",   width // 4, 3 * width // 4, 0, height // 4, (255, 0, 0)),
            ("Right",  width // 2, width, 0, height // 4, (0, 165, 255))
        ]

        target_pt = None
        target_color = (0, 0, 255)
        
        # Check ROIs in priority order
        # But wait, user said "try to match main centroid to BLUE centroid"
        # AND "decide left... otherwise right".
        # This implies: If Left exists, use it. If not, use Blue? 
        # Or: Use Blue normally. But at junctions (implies Left and Right/Blue exist), pick Left.
        
        # Let's collect all valid centroids first
        centroids = {}
        for name, r_x1, r_x2, r_y1, r_y2, color in rois:
            roi_mask = mask[r_y1:r_y2, r_x1:r_x2]
            c_list, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if c_list:
                c = max(c_list, key=cv2.contourArea)
                if cv2.contourArea(c) > 50: # Minimal area check
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) + r_x1
                        cy = int(M["m01"] / M["m00"]) + r_y1
                        centroids[name] = (cx, cy)

        # Decision Logic:
        # If "Left" exists -> Go Left
        # Else If "Blue" (Middle) exists -> Go Blue
        # Else If "Right" exists -> Go Right
        # Else -> U-Turn
        
        if "Left" in centroids:
            target_pt = centroids["Left"]
            target_color = rois[0][5] # Green
        elif "Blue" in centroids:
            target_pt = centroids["Blue"]
            target_color = rois[1][5] # Red/Blue
        elif "Right" in centroids:
            target_pt = centroids["Right"]
            target_color = rois[2][5] # Orange
        
        self.debug_info = {
            "mask": mask,
            "img": frame.copy(),
            "target": target_pt,
            "fixed": fixed_pt,
            "rois": rois,
            "centroids": centroids
        }

        if target_pt:
            # PID Control on Heading
            error_x = fixed_pt[0] - target_pt[0]
            omega = self.Kp * error_x
            vx = self.speed
            vy = 0.0
            self.last_target = target_pt
            return vx, vy, omega
        else:
            # NO CENTROIDS FOUND
            # Debug empty state
            self.last_target = None
            return 0.0, 0.0, 0.1  # Slow Scan

    def get_debug_frame(self) -> np.ndarray:
        info = self.debug_info
        out = info["img"].copy()
        mask = info["mask"]
        
        # Enlarge for visibility
        out = cv2.resize(out, (400, 400), interpolation=cv2.INTER_NEAREST)
        mask_display = cv2.resize(mask, (400, 400), interpolation=cv2.INTER_NEAREST)
        
        scale = 400 / FRAME_SIZE
        def s(pt): return (int(pt[0] * scale), int(pt[1] * scale))
        
        # Blend mask
        mask_bgr = cv2.cvtColor(mask_display, cv2.COLOR_GRAY2BGR)
        out = cv2.addWeighted(out, 0.7, mask_bgr, 0.3, 0)
        
        # Draw grid

        # Draw ROIs
        for name, x1, x2, y1, y2, color in info["rois"]:
            cv2.rectangle(out, s((x1, y1)), s((x2, y2)), color, 2)
            cv2.putText(out, name, s((x1, y1+5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1) # Label ROI

        # Draw centroids
        for name, pt in info["centroids"].items():
            cv2.circle(out, s(pt), 6, (0, 255, 255), -1) # Yellow dots
            # Label centroid
            cv2.putText(out, name, (s(pt)[0]+10, s(pt)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Draw fixed
        cv2.circle(out, s(info["fixed"]), 8, (0, 0, 255), -1) # Red fixed
        
        # Draw target link
        if info["target"]:
            cv2.circle(out, s(info["target"]), 8, (255, 0, 0), -1) # Blue target
            cv2.line(out, s(info["fixed"]), s(info["target"]), (0, 255, 0), 2)
        else:
            cv2.putText(out, "NO TARGET", (10, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        return out

def main():
    # 1) controller (Simulated)
    controller = FrameSimulator(
        MAP_PATH,
        frame_size=FRAME_SIZE,
        start_x=START_X,
        start_y=START_Y,
        start_theta=START_THETA
    )

    # 2) logic
    follower = LineFollower()

    delay = int(1000 / FPS)
    print("IARC Line Follower — press 'q' to quit, 'p' to pause/resume")

    paused = False

    while True:
        # grab frame (rotated)
        frame = controller.get_frame()
        # ret, frame = camera.read()



        if not paused:
            # decide velocity
            vx, vy, omega = follower.process(frame)

            control_bot(vx, vy, omega)


            # send to controller
            controller.step(vx, vy, omega)


        # ── visualise ─────────────────────────────────────────────────
        # controller.show()                                   # map view
        if hasattr(follower, 'debug_info') and follower.debug_info:
            debug = follower.get_debug_frame()
            if paused:
                cv2.putText(debug, "PAUSED", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.imshow("Robot View", debug)                     # robot view

        pos = controller.get_position() # (x, y, theta)
        cv2.setWindowTitle("FrameSimulator",
                           f"Pos=({pos[0]:.0f},{pos[1]:.0f}) Theta={np.degrees(pos[2]):.0f} deg" + (" [PAUSED]" if paused else ""))

        key = cv2.waitKey(delay) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("p"):
            paused = not paused

    controller.close()

if __name__ == "__main__":
    main()