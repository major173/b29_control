#!/usr/bin/env python3
"""Publish a simple support bar collision object into the MoveIt planning scene."""

import rospy
import tf.transformations as tf_trans

try:
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    MOVEIT_IMPORT_ERROR = None
except ImportError as exc:  # pragma: no cover - depends on optional MoveIt install
    Pose = None
    CollisionObject = None
    PlanningScene = None
    SolidPrimitive = None
    MOVEIT_IMPORT_ERROR = exc


class SupportBarPublisher:
    def __init__(self):
        self._publisher = rospy.Publisher("/planning_scene", PlanningScene, queue_size=1, latch=True)
        self._object_id = rospy.get_param("~object_id", "support_bar")
        self._frame_id = rospy.get_param("~frame_id", "world")
        self._radius = float(rospy.get_param("~radius", 0.007))
        self._length = float(rospy.get_param("~length", 3.0))
        self._xyz = [
            float(rospy.get_param("~x", 0.0)),
            float(rospy.get_param("~y", 1.002)),
            float(rospy.get_param("~z", 2.0)),
        ]
        self._rpy = [
            float(rospy.get_param("~roll", 1.5707963267948966)),
            float(rospy.get_param("~pitch", 0.0)),
            float(rospy.get_param("~yaw", 0.0)),
        ]
        self._publish_period = max(0.5, float(rospy.get_param("~publish_period", 2.0)))
        self._timer = rospy.Timer(rospy.Duration(self._publish_period), self._publish)
        self._publish(None)

    def _build_scene(self):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [self._length, self._radius]

        pose = Pose()
        pose.position.x = self._xyz[0]
        pose.position.y = self._xyz[1]
        pose.position.z = self._xyz[2]
        quaternion = tf_trans.quaternion_from_euler(*self._rpy)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        collision_object = CollisionObject()
        collision_object.header.frame_id = self._frame_id
        collision_object.id = self._object_id
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(collision_object)
        return planning_scene

    def _publish(self, _event):
        planning_scene = self._build_scene()
        self._publisher.publish(planning_scene)


def main():
    rospy.init_node("gp11_moveit_scene_loader")
    if MOVEIT_IMPORT_ERROR is not None:
        rospy.logerr(
            "MoveIt Python messages are unavailable: %s. Install MoveIt before running this node.",
            MOVEIT_IMPORT_ERROR,
        )
        return
    SupportBarPublisher()
    rospy.loginfo("GP11 MoveIt support bar collision object publisher started")
    rospy.spin()


if __name__ == "__main__":
    main()
