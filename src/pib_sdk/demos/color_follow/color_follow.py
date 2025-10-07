import cv2
import numpy as np
import time
from typing import Tuple, Optional
from pib_sdk.control import Write, default
from pib_sdk.speech import speak  # <-- TTS

# ----------------- HSV RANGES (OpenCV hue: 0..180) -----------------
HSV_RANGES = {
    "blue":   [(np.array([100, 120,  70]), np.array([130, 255, 255]))],
    "green":  [(np.array([ 40,  80,  70]), np.array([ 80, 255, 255]))],
    "orange": [(np.array([ 10, 120,  70]), np.array([ 25, 255, 255]))],
    "pink":   [(np.array([145,  80,  70]), np.array([170, 255, 255]))],
    "red":    [
        (np.array([  0, 120,  70]), np.array([ 10, 255, 255])),
        (np.array([170, 120,  70]), np.array([180, 255, 255]))
    ],
}

def _largest_centroid(mask: np.ndarray, min_area: int = 500) -> Optional[Tuple[int,int,int]]:
    kernel = np.ones((5,5), np.uint8)
    clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    contours, _ = cv2.findContours(clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = int(cv2.contourArea(c))
    if area < min_area:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy, area

def _norm_to_degrees(cx: int, cy: int, w: int, h: int,
                     max_deg: float = 30.0, kx: float = 1.0, ky: float = 1.0) -> Tuple[float, float]:
    nx = (cx - (w/2.0)) / (w/2.0)   # left -1 ? right +1
    ny = (cy - (h/2.0)) / (h/2.0)   # up -1 ? down +1
    yaw   = float(np.clip(kx * nx * max_deg, -max_deg, max_deg))
    pitch = float(np.clip(-ky * ny * max_deg, -max_deg, max_deg))  # screen down ? tilt down (+)
    return yaw, pitch

def _build_oak_pipeline():
    import depthai as dai
    pipe = dai.Pipeline()
    cam = pipe.create(dai.node.ColorCamera)
    cam.setPreviewSize(640, 480)
    cam.setInterleaved(False)
    cam.setFps(30)

    xout = pipe.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)
    return pipe

def _oak_frames():
    import depthai as dai
    pipe = _build_oak_pipeline()
    with dai.Device(pipe) as dev:
        q = dev.getOutputQueue("rgb", maxSize=4, blocking=False)
        while True:
            pkt = q.get()
            frame = pkt.getCvFrame()
            yield frame

def main(kwargs) -> int:
    # CLI args mapped in registry/CLI
    color    = (kwargs.get("color") or "blue").lower()
    show     = bool(kwargs.get("show", False))
    host     = kwargs.get("host", "localhost")
    port     = int(kwargs.get("port", 9090))
    debug    = bool(kwargs.get("debug", False))
    max_deg  = float(kwargs.get("max_deg", 30.0))
    min_area = int(kwargs.get("min_area", 600))

    ranges = HSV_RANGES.get(color)
    if not ranges:
        print(f"[color_follow] Unsupported color '{color}'. Supported: {', '.join(HSV_RANGES.keys())}")
        return 2

    w = None
    s = None
    try:
        # ROS control + speech
        w = Write(host=host, port=port, debug=debug)
        s = speak(host=host, port=port, debug=debug)

        # Prep head motors with defaults but faster velocity
        w.set("turn_head_motor", "tilt_forward_motor", default, velocity=20000)

        last_send = 0.0
        send_hz   = 15.0
        dt_min    = 1.0 / send_hz

        # announce only when we *first* see the color (or after being lost for a while)
        seen_recently = False
        lost_since: Optional[float] = None
        reacquire_silence = 1.5  # must be unseen for this long to re-announce

        for frame in _oak_frames():
            h, wimg = frame.shape[:2]
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, lo, hi)
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            target = _largest_centroid(mask, min_area=min_area)
            if target:
                # (Re)acquire announcement
                if not seen_recently:
                    # If we were lost, check the silent gap
                    if (lost_since is None) or ((time.time() - lost_since) >= reacquire_silence):
                        try:
                            s.say(f'I am seeing "{color}" object and following it now.', join=False)
                        except Exception as e:
                            if debug:
                                print(f"[speech] say error: {e}")
                    seen_recently = True
                    lost_since = None

                cx, cy, area = target
                yaw_deg, pitch_deg = _norm_to_degrees(cx, cy, wimg, h, max_deg=max_deg)

                t = time.time()
                if (t - last_send) >= dt_min:
                    _ = w.move("turn_head_motor", yaw_deg, "tilt_forward_motor", pitch_deg)
                    last_send = t

                if show:
                    cv2.circle(frame, (cx, cy), 8, (0,255,0), 2)
                    cv2.putText(frame, f"{color} A={area} yaw={yaw_deg:.1f} pitch={pitch_deg:.1f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                if seen_recently:
                    seen_recently = False
                    lost_since = time.time()
                if show:
                    cv2.putText(frame, f"looking for {color}...", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            if show:
                cv2.imshow("pib color_follow (OAK)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):   # <-- quit on 'q'
                    break

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if w and hasattr(w, "close"):
                w.close()
        except Exception:
            pass
        try:
            if s:
                s.close()
        except Exception:
            pass
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
    return 0

if __name__ == "__main__":
    import argparse

    def str2bool(v: str) -> bool:
        if isinstance(v, bool):
            return v
        v = v.lower()
        if v in ("yes", "true", "t", "1", "y", "on"):
            return True
        if v in ("no", "false", "f", "0", "n", "off"):
            return False
        raise argparse.ArgumentTypeError("Expected a boolean value.")

    parser = argparse.ArgumentParser(description="pib color-follow (OAK)")
    parser.add_argument("--color", default="blue", help="blue/green/orange/pink/red")
    parser.add_argument("--show", nargs="?", const=True, default=False, type=str2bool,
                        help="Show camera window (default: false). Use '--show' or '--show true'.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--max-deg", dest="max_deg", type=float, default=30.0)
    parser.add_argument("--min-area", dest="min_area", type=int, default=600)

    args = parser.parse_args()
    raise SystemExit(main(vars(args)))
