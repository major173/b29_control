#!/usr/bin/env python3
"""
anchor_world_tf_publisher.py

以固定端 anchor_link 为世界基座，动态发布：
  world → base_link          (使 RViz 以固定端为参考系)
  world → capture_output_ref (capture 输出参考坐标系，与 MuJoCo 对齐)

capture 输出参考坐标系定义（与 gp11_capture_runtime.py 一致）：
  origin = output_orientation_body.position + signed_distance * body_x_axis
  axes   = output_orientation_body 的旋转矩阵
  signed_distance:
    anchor_side=right → output_orientation_body=right_second_leg, +0.7401573658 m
    anchor_side=left  → output_orientation_body=left_second_leg,  -0.7401573658 m

参数：
  ~anchor_link   : 固定端 link，默认 left_second_leg
  ~anchor_side   : right 或 left，决定 output_orientation_body 和符号
  ~anchor_x/y/z  : world → anchor 的平移，默认 0
  ~anchor_roll/pitch/yaw : world → anchor 的旋转，默认 0
  ~rate          : 发布频率 Hz，默认 50
"""

import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3

_OUTPUT_REFERENCE_DISTANCE = 0.7401573658

_SIDE_SPECS = {
    "right": ("right_second_leg", +1.0),
    "left":  ("left_second_leg",  -1.0),
}


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _qmul(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a.x, a.y, a.z, a.w
    bx, by, bz, bw = b.x, b.y, b.z, b.w
    return Quaternion(
        x=aw * bx + ax * bw + ay * bz - az * by,
        y=aw * by - ax * bz + ay * bw + az * bx,
        z=aw * bz + ax * by - ay * bx + az * bw,
        w=aw * bw - ax * bx - ay * by - az * bz,
    )


def _qrot(q: Quaternion, v: Vector3) -> Vector3:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix = qw * v.x + qy * v.z - qz * v.y
    iy = qw * v.y + qz * v.x - qx * v.z
    iz = qw * v.z + qx * v.y - qy * v.x
    iw = -qx * v.x - qy * v.y - qz * v.z
    return Vector3(
        x=ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy),
        y=iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz),
        z=iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx),
    )


def _mul_tf(t1: TransformStamped, t2: TransformStamped) -> TransformStamped:
    """t_out = t1 * t2"""
    rotated = _qrot(t1.transform.rotation, t2.transform.translation)
    out = TransformStamped()
    out.header.frame_id = t1.header.frame_id
    out.child_frame_id  = t2.child_frame_id
    out.transform.translation.x = t1.transform.translation.x + rotated.x
    out.transform.translation.y = t1.transform.translation.y + rotated.y
    out.transform.translation.z = t1.transform.translation.z + rotated.z
    out.transform.rotation = _qmul(t1.transform.rotation, t2.transform.rotation)
    return out


def _make_tf(parent, child, tx, ty, tz, qx, qy, qz, qw) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id  = child
    t.transform.translation.x = tx
    t.transform.translation.y = ty
    t.transform.translation.z = tz
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw
    return t


def _output_ref_from_body_tf(body_tf: TransformStamped, signed_distance: float,
                              parent_frame: str) -> TransformStamped:
    """
    从 body TF 计算 capture 输出参考坐标系 TF。
    origin = body_pos + signed_distance * body_x_axis
    axes   = body 旋转（与 MuJoCo _capture_output_frame_from_body 一致）
    """
    q = body_tf.transform.rotation
    # body x 轴在父坐标系下 = R * [1,0,0]
    x_axis = _qrot(q, Vector3(x=1.0, y=0.0, z=0.0))
    tr = body_tf.transform.translation
    out = TransformStamped()
    out.header.frame_id = parent_frame
    out.child_frame_id  = "capture_output_ref"
    out.transform.translation.x = tr.x + signed_distance * x_axis.x
    out.transform.translation.y = tr.y + signed_distance * x_axis.y
    out.transform.translation.z = tr.z + signed_distance * x_axis.z
    out.transform.rotation = q  # 朝向与 body 一致
    return out


def main():
    rospy.init_node("anchor_world_tf_publisher", anonymous=False)

    anchor_link  = rospy.get_param("~anchor_link", "left_second_leg")
    anchor_side  = rospy.get_param("~anchor_side", "left")
    rate_hz      = rospy.get_param("~rate", 50.0)
    ax = rospy.get_param("~anchor_x",     0.0)
    ay = rospy.get_param("~anchor_y",     0.0)
    az = rospy.get_param("~anchor_z",     0.0)
    ar = rospy.get_param("~anchor_roll",  0.0)
    ap = rospy.get_param("~anchor_pitch", 0.0)
    aw_param = rospy.get_param("~anchor_yaw", 0.0)

    # anchor_side=base 时，base_link 已被 Gazebo 固定在 world
    # 发布静态 world → base_link identity TF，使完整 TF 链可用
    if anchor_side == "base":
        rospy.loginfo("[anchor_world_tf] anchor_side=base, publishing static world→base_link identity TF")
        static_br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        static_br.sendTransform(t)
        rospy.spin()
        return

    output_body, sign = _SIDE_SPECS.get(anchor_side, _SIDE_SPECS["left"])
    signed_distance = sign * _OUTPUT_REFERENCE_DISTANCE

    qx, qy, qz, qw = _euler_to_quat(ar, ap, aw_param)
    world_to_anchor = _make_tf("world", anchor_link, ax, ay, az, qx, qy, qz, qw)

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    br = tf2_ros.TransformBroadcaster()

    static_br = tf2_ros.StaticTransformBroadcaster()
    world_to_anchor.header.stamp = rospy.Time.now()
    static_br.sendTransform(world_to_anchor)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo(
        "[anchor_world_tf] anchor=%s  side=%s  output_body=%s  signed_dist=%.4f m",
        anchor_link, anchor_side, output_body, signed_distance,
    )

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        try:
            # world → base_link
            anchor_to_base = buf.lookup_transform(
                anchor_link, "base_link", rospy.Time(0),
                timeout=rospy.Duration(0.05),
            )
            world_to_base = _mul_tf(world_to_anchor, anchor_to_base)
            world_to_base.header.stamp    = now
            world_to_base.header.frame_id = "world"
            world_to_base.child_frame_id  = "base_link"
            br.sendTransform(world_to_base)

            # world → capture_output_ref
            # 先查 world → output_body（通过已发布的 world→base_link 链路）
            world_to_output_body = buf.lookup_transform(
                "world", output_body, rospy.Time(0),
                timeout=rospy.Duration(0.05),
            )
            output_ref_tf = _output_ref_from_body_tf(
                world_to_output_body, signed_distance, "world"
            )
            output_ref_tf.header.stamp = now
            br.sendTransform(output_ref_tf)

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            pass
        rate.sleep()


if __name__ == "__main__":
    main()
