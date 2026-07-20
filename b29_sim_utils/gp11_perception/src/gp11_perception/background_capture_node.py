"""Save one depth frame from a real camera as a background model."""

from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _depth_to_meters(msg, bridge):
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    if msg.encoding in ("16UC1", "mono16"):
        return image.astype(np.float32) * 0.001
    return image.astype(np.float32)


def main():
    rospy.init_node("gp11_capture_depth_background")
    depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
    output_path_raw = str(rospy.get_param("~output_path", "")).strip()
    png_preview_path = rospy.get_param("~preview_png_path", "")
    timeout = float(rospy.get_param("~timeout", 10.0))

    if not output_path_raw:
        raise RuntimeError("~output_path 不能为空")
    output_path = Path(output_path_raw).expanduser().resolve()

    rospy.loginfo("Waiting for one depth frame from %s", depth_topic)
    msg = rospy.wait_for_message(depth_topic, Image, timeout=timeout)
    bridge = CvBridge()
    depth_m = _depth_to_meters(msg, bridge)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(str(output_path), depth_m)
    rospy.loginfo("Saved background depth to %s", output_path)

    if png_preview_path:
        preview_path = Path(png_preview_path).expanduser().resolve()
        preview_path.parent.mkdir(parents=True, exist_ok=True)
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        preview = np.zeros_like(depth_m, dtype=np.uint8)
        if np.any(valid):
            values = depth_m[valid]
            lo = float(np.percentile(values, 5))
            hi = float(np.percentile(values, 95))
            scale = max(1e-6, hi - lo)
            preview[valid] = np.clip((depth_m[valid] - lo) * 255.0 / scale, 0.0, 255.0).astype(np.uint8)
        cv2.imwrite(str(preview_path), preview)
        rospy.loginfo("Saved preview PNG to %s", preview_path)
