#!/usr/bin/env python3
"""
anchor_world_tf_publisher.py

以固定端 anchor_link（默认 left_second_leg）为世界基座，
动态发布 world → base_link TF，使 RViz 以固定端为参考系显示。

原理：
  world → anchor_link  (静态，可配置，默认 identity)
  anchor_link → base_link  (从 TF buffer 反向查，TF2 支持双向遍历)
  → 发布 world → base_link = world→anchor × anchor→base_link

适用于 Gazebo 仿真和实物部署，无需 /gazebo/link_states。

参数：
  ~anchor_link   : 固定端 link 名，默认 left_second_leg
  ~anchor_x/y/z  : world → anchor 的平移，默认 0
  ~anchor_roll/pitch/yaw : world → anchor 的旋转，默认 0
  ~rate          : 发布频率 Hz，默认 50
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


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


def _mul_transform(t1: TransformStamped, t2: TransformStamped) -> TransformStamped:
    """t_out = t1 * t2  (t1.child == t2.header.frame_id)"""
    from geometry_msgs.msg import Vector3, Quaternion

    def qmul(a, b):
        ax, ay, az, aw = a.x, a.y, a.z, a.w
        bx, by, bz, bw = b.x, b.y, b.z, b.w
        return Quaternion(
            x=aw * bx + ax * bw + ay * bz - az * by,
            y=aw * by - ax * bz + ay * bw + az * bx,
            z=aw * bz + ax * by - ay * bx + az * bw,
            w=aw * bw - ax * bx - ay * by - az * bz,
        )

    def qrot(q, v):
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

    t1_tr = t1.transform.translation
    t2_tr = t2.transform.translation
    t1_q  = t1.transform.rotation
    rotated = qrot(t1_q, t2_tr)

    out = TransformStamped()
    out.header.frame_id  = t1.header.frame_id
    out.child_frame_id   = t2.child_frame_id
    out.transform.translation.x = t1_tr.x + rotated.x
    out.transform.translation.y = t1_tr.y + rotated.y
    out.transform.translation.z = t1_tr.z + rotated.z
    out.transform.rotation = qmul(t1_q, t2.transform.rotation)
    return out


def main():
    rospy.init_node("anchor_world_tf_publisher", anonymous=False)

    anchor_link = rospy.get_param("~anchor_link", "left_second_leg")
    rate_hz     = rospy.get_param("~rate", 50.0)
    ax = rospy.get_param("~anchor_x",   0.0)
    ay = rospy.get_param("~anchor_y",   0.0)
    az = rospy.get_param("~anchor_z",   0.0)
    ar = rospy.get_param("~anchor_roll",  0.0)
    ap = rospy.get_param("~anchor_pitch", 0.0)
    aw = rospy.get_param("~anchor_yaw",   0.0)

    # world → anchor_link (静态)
    qx, qy, qz, qw = _euler_to_quat(ar, ap, aw)
    world_to_anchor = TransformStamped()
    world_to_anchor.header.frame_id = "world"
    world_to_anchor.child_frame_id  = anchor_link
    world_to_anchor.transform.translation.x = ax
    world_to_anchor.transform.translation.y = ay
    world_to_anchor.transform.translation.z = az
    world_to_anchor.transform.rotation.x = qx
    world_to_anchor.transform.rotation.y = qy
    world_to_anchor.transform.rotation.z = qz
    world_to_anchor.transform.rotation.w = qw

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    br  = tf2_ros.TransformBroadcaster()

    # 发布静态 world → anchor_link
    static_br = tf2_ros.StaticTransformBroadcaster()
    world_to_anchor.header.stamp = rospy.Time.now()
    static_br.sendTransform(world_to_anchor)

    rate = rospy.Rate(rate_hz)
    rospy.loginfo(
        "[anchor_world_tf] anchor=%s  world_origin=(%.3f, %.3f, %.3f)",
        anchor_link, ax, ay, az,
    )

    while not rospy.is_shutdown():
        try:
            # anchor_link → base_link（TF2 反向遍历 base_link→...→anchor_link）
            anchor_to_base = buf.lookup_transform(
                anchor_link, "base_link", rospy.Time(0),
                timeout=rospy.Duration(0.1),
            )
            world_to_base = _mul_transform(world_to_anchor, anchor_to_base)
            world_to_base.header.stamp    = rospy.Time.now()
            world_to_base.header.frame_id = "world"
            world_to_base.child_frame_id  = "base_link"
            br.sendTransform(world_to_base)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            pass
        rate.sleep()


if __name__ == "__main__":
    main()
