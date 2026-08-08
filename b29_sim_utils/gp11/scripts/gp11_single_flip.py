#!/usr/bin/env python3
"""Manually start one flip after an independently verified disconnect."""

import sys

import rospy

from gp11.single_flip_client import SingleFlipClient


def main():
    rospy.init_node("gp11_single_flip", anonymous=True)
    return SingleFlipClient().run()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(130)
