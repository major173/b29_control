#!/usr/bin/env python3
"""Generate a planning-only URDF rooted at a chosen support link."""

import argparse
import copy
import math
import sys
import xml.etree.ElementTree as ET

import numpy
from tf.transformations import euler_from_matrix, euler_matrix, inverse_matrix, rotation_matrix, translation_from_matrix, translation_matrix


MOVING_JOINT_TYPES = {"continuous", "revolute", "prismatic"}


def parse_xyz(text):
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError("Expected 3 values, got {!r}".format(text))
    return numpy.array(values, dtype=float)


def matrix_from_origin(origin_element):
    if origin_element is None:
        xyz = numpy.zeros(3, dtype=float)
        rpy = numpy.zeros(3, dtype=float)
    else:
        xyz = parse_xyz(origin_element.get("xyz", "0 0 0"))
        rpy = parse_xyz(origin_element.get("rpy", "0 0 0"))
    return numpy.dot(translation_matrix(xyz), euler_matrix(*rpy))


def axis_from_joint(joint_element):
    axis_element = joint_element.find("axis")
    if axis_element is None:
        return numpy.array((1.0, 0.0, 0.0), dtype=float)
    return parse_xyz(axis_element.get("xyz", "1 0 0"))


def matrix_to_xyz_rpy(matrix):
    xyz = translation_from_matrix(matrix)
    rpy = euler_from_matrix(matrix)
    xyz_text = "{:.9f} {:.9f} {:.9f}".format(*xyz)
    rpy_text = "{:.9f} {:.9f} {:.9f}".format(*rpy)
    return xyz_text, rpy_text


def joint_motion_matrix(joint_element, position):
    joint_type = joint_element.get("type", "fixed")
    axis = axis_from_joint(joint_element)
    if joint_type in ("continuous", "revolute"):
        return rotation_matrix(position, axis)
    if joint_type == "prismatic":
        return translation_matrix(axis * position)
    return numpy.identity(4)


def joint_static_matrix(joint_element):
    return matrix_from_origin(joint_element.find("origin"))


def joint_transform(joint_element, position):
    return numpy.dot(joint_static_matrix(joint_element), joint_motion_matrix(joint_element, position))


def copy_non_kinematic_children(source_joint, target_joint):
    for child in list(source_joint):
        if child.tag in ("parent", "child", "origin", "axis"):
            continue
        target_joint.append(copy.deepcopy(child))


def make_origin_element(matrix):
    xyz_text, rpy_text = matrix_to_xyz_rpy(matrix)
    origin = ET.Element("origin")
    origin.set("xyz", xyz_text)
    origin.set("rpy", rpy_text)
    return origin


def make_parent_child(parent_link, child_link):
    parent = ET.Element("parent")
    parent.set("link", parent_link)
    child = ET.Element("child")
    child.set("link", child_link)
    return parent, child


def make_fixed_joint(name, parent_link, child_link, matrix):
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    parent, child = make_parent_child(parent_link, child_link)
    joint.append(parent)
    joint.append(child)
    joint.append(make_origin_element(matrix))
    return joint


def make_inverse_active_joint(source_joint, parent_link, child_link, axis_sign=-1.0):
    joint = ET.Element("joint", {"name": source_joint.get("name"), "type": source_joint.get("type", "fixed")})
    parent, child = make_parent_child(parent_link, child_link)
    joint.append(parent)
    joint.append(child)
    joint.append(make_origin_element(numpy.identity(4)))

    # A normally re-rooted active joint must reverse its axis.  Some real
    # mechanisms have a calibrated model-to-anchor direction correction; keep
    # that correction private to the generated planning URDF rather than
    # changing B29's shared URDF, encoder feedback or hardware protocol.
    axis_values = float(axis_sign) * axis_from_joint(source_joint)
    axis = ET.Element("axis")
    axis.set("xyz", "{:.9f} {:.9f} {:.9f}".format(*axis_values))
    joint.append(axis)

    copy_non_kinematic_children(source_joint, joint)
    return joint


def compute_chain_transform(anchor_link, child_to_joint, joint_positions):
    transform = numpy.identity(4)
    current = anchor_link
    chain = []
    while current in child_to_joint:
        joint_element = child_to_joint[current]
        chain.append(joint_element)
        current = joint_element.find("parent").get("link")
    chain.reverse()
    for joint_element in chain:
        position = joint_positions.get(joint_element.get("name"), 0.0)
        transform = numpy.dot(transform, joint_transform(joint_element, position))
    return transform


def build_rerooted_robot(
        source_robot, anchor_link, joint_positions, world_matrix=None,
        anchor_matrix=None, inverse_joint_axis_signs=None):
    # /robot_description may already contain a world root from a previous
    # fixed-base or anchored model.  This generator owns the planning-only
    # world root, so discard the old boundary before rebuilding it.  Copying
    # it would create two <link name="world"> elements and MoveIt refuses to
    # parse the resulting URDF.
    links = {
        link.get("name"): link
        for link in source_robot.findall("link")
        if link.get("name") != "world"
    }
    inverse_joint_axis_signs = inverse_joint_axis_signs or {}
    joints = [
        joint for joint in source_robot.findall("joint")
        if joint.find("parent").get("link") != "world"
        and joint.find("child").get("link") != "world"
    ]

    if anchor_link not in links:
        raise RuntimeError("Anchor link '{}' does not exist in source robot".format(anchor_link))

    child_to_joint = {}
    adjacency = {link_name: [] for link_name in links}
    for joint_element in joints:
        parent_link = joint_element.find("parent").get("link")
        child_link = joint_element.find("child").get("link")
        child_to_joint[child_link] = joint_element
        adjacency[parent_link].append((joint_element, child_link, "forward"))
        adjacency[child_link].append((joint_element, parent_link, "inverse"))

    target_robot = ET.Element("robot", {"name": source_robot.get("name", "gp11")})
    target_robot.append(ET.Element("link", {"name": "world"}))

    if anchor_matrix is not None:
        world_to_anchor = anchor_matrix
    else:
        if world_matrix is None:
            raise RuntimeError("Either world_matrix or anchor_matrix must be provided")
        base_to_anchor = compute_chain_transform(anchor_link, child_to_joint, joint_positions)
        world_to_anchor = numpy.dot(world_matrix, base_to_anchor)
    target_robot.append(make_fixed_joint("world_to_anchor", "world", anchor_link, world_to_anchor))

    for link_name in links:
        target_robot.append(copy.deepcopy(links[link_name]))

    dummy_links = []
    target_joints = []
    visited_links = {anchor_link}
    stack = [anchor_link]

    while stack:
        current_link = stack.pop()
        for joint_element, neighbor_link, direction in adjacency[current_link]:
            if neighbor_link in visited_links:
                continue

            joint_name = joint_element.get("name")
            joint_type = joint_element.get("type", "fixed")
            fixed_position = joint_positions.get(joint_name)

            if direction == "forward":
                if joint_type in MOVING_JOINT_TYPES and fixed_position is not None:
                    target_joints.append(
                        make_fixed_joint(joint_name, current_link, neighbor_link, joint_transform(joint_element, fixed_position))
                    )
                else:
                    target_joints.append(copy.deepcopy(joint_element))
            else:
                if joint_type in MOVING_JOINT_TYPES and fixed_position is None:
                    dummy_name = "{}__inv_link".format(joint_name)
                    dummy_links.append(ET.Element("link", {"name": dummy_name}))
                    target_joints.append(
                        make_inverse_active_joint(
                            joint_element,
                            current_link,
                            dummy_name,
                            inverse_joint_axis_signs.get(joint_name, -1.0),
                        )
                    )
                    target_joints.append(
                        make_fixed_joint(
                            "{}__inv_offset".format(joint_name),
                            dummy_name,
                            neighbor_link,
                            inverse_matrix(joint_static_matrix(joint_element)),
                        )
                    )
                else:
                    position = fixed_position if fixed_position is not None else 0.0
                    target_joints.append(
                        make_fixed_joint(
                            joint_name,
                            current_link,
                            neighbor_link,
                            inverse_matrix(joint_transform(joint_element, position)),
                        )
                    )

            visited_links.add(neighbor_link)
            stack.append(neighbor_link)

    for link_element in dummy_links:
        target_robot.append(link_element)
    for joint_element in target_joints:
        target_robot.append(joint_element)

    return target_robot


def parse_joint_position(item):
    name, value = item.split("=", 1)
    return name.strip(), float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Path to the source URDF file.")
    parser.add_argument("--anchor-link", required=True, help="Link that should become world-anchored.")
    parser.add_argument("--base-x", type=float, default=0.03)
    parser.add_argument("--base-y", type=float, default=1.002)
    parser.add_argument("--base-z", type=float, default=1.55)
    parser.add_argument("--base-yaw", type=float, default=-1.5707963)
    parser.add_argument("--anchor-x", type=float, default=None)
    parser.add_argument("--anchor-y", type=float, default=None)
    parser.add_argument("--anchor-z", type=float, default=None)
    parser.add_argument("--anchor-roll", type=float, default=0.0)
    parser.add_argument("--anchor-pitch", type=float, default=0.0)
    parser.add_argument("--anchor-yaw", type=float, default=0.0)
    parser.add_argument(
        "--fixed-joint",
        action="append",
        default=[],
        help="Freeze a joint at the given value, for example joint_name=0.005",
    )
    args = parser.parse_args()

    source_robot = ET.parse(args.input).getroot()
    joint_positions = dict(parse_joint_position(item) for item in args.fixed_joint)
    anchor_matrix = None
    if args.anchor_x is not None and args.anchor_y is not None and args.anchor_z is not None:
        anchor_matrix = numpy.dot(
            translation_matrix((args.anchor_x, args.anchor_y, args.anchor_z)),
            euler_matrix(args.anchor_roll, args.anchor_pitch, args.anchor_yaw),
        )

    world_matrix = None
    if anchor_matrix is None:
        world_matrix = numpy.dot(
            translation_matrix((args.base_x, args.base_y, args.base_z)),
            euler_matrix(0.0, 0.0, args.base_yaw),
        )

    target_robot = build_rerooted_robot(
        source_robot,
        args.anchor_link,
        joint_positions,
        world_matrix=world_matrix,
        anchor_matrix=anchor_matrix,
    )
    xml_text = ET.tostring(target_robot, encoding="unicode")
    sys.stdout.write(xml_text)


if __name__ == "__main__":
    main()
