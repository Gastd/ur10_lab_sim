#!/usr/bin/env python3

"""
Moving to a fix set of joint configurations
to scan an object.
Inspired by the example ex_pose_goal.py of pymoveit2 (https://github.com/AndrejOrsula/pymoveit2/)
"""

from threading import Thread
import random
import math
import json
from math import sin, cos

from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State



def main():
    rclpy.init()

    # with open("fixed_positions.json","r") as f:
    #     D = json.load(f)
    #     fixed_poses = D["joint_positions"]

    fixed_poses = [
        [-0.45189339575543486, -1.4339117226438687, -1.4957283295398132, -1.2379874027427842, 0.7542135741036173, 0.8792894677022602],
        [-1.1074071691742848, -1.1516038541293385, -2.191649335071931, -0.47737865696203546, 1.2009377367225342, 0.28586365028094574],
        [-0.7554219744492617, -1.5038943312932598, -1.9382617963861122, -1.4015408899210067, 0.46275206040547023, 1.2758021492294673],
        [-0.95121465749664, -1.9985839825564922, -1.5190273940958223, -1.8643584263686417, 0.8165665466823404, 2.127449232196525],
        [-1.3799200304821821, -2.4100364483103602, -0.6648184473117609, -2.3155779270809327, 1.4749482273456205, 1.375197232970259],
        [-1.789081750844519, -2.1674764289478077, -0.9468283864878606, -1.814372863122594, 2.307844779131693, 0.4033143311933179],
        [-2.0532635686390197, -1.6533233974218908, -1.9063357107175596, -0.6287754470522707, 2.142191920203904, 0.5781473765181401],
    ]

    # Create node for this example
    node = Node("scan")

    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    # Declare parameter for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            -0.7853981633974483,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ],
    )
    node.declare_parameter("synchronous", True)
    # If non-positive, don't cancel. Only used if synchronous is False
    node.declare_parameter("cancel_after_secs", 0.0)
    # Planner ID
    node.declare_parameter("planner_id", "RRTConnectkConfigDefault")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    node.get_logger().info(f"Moving ur_manipulator")
    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
        callback_group=callback_group,
    )
    moveit2.planner_id = (
        node.get_parameter("planner_id").get_parameter_value().string_value
    )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Scale down velocity and acceleration of joints (percentage of maximum)
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Get parameters
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value
    cancel_after_secs = (
        node.get_parameter("cancel_after_secs").get_parameter_value().double_value
    )

    # Set some planner params
    cartesian = False
    cartesian_max_step = 0.0025
    cartesian_fraction_threshold = 0.0
    # Set parameters for cartesian planning
    moveit2.cartesian_avoid_collisions = False
    moveit2.cartesian_jump_threshold = 0.0

    for joint_positions in fixed_poses:
        # Move to joint configuration
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        moveit2.move_to_configuration(joint_positions)

        if synchronous:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            moveit2.wait_until_executed()
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            print("Current State: " + str(moveit2.query_state()))
            rate = node.create_rate(10)
            while moveit2.query_state() != MoveIt2State.EXECUTING:
                rate.sleep()

            # Get the future
            print("Current State: " + str(moveit2.query_state()))
            future = moveit2.get_execution_future()

            # Cancel the goal
            if cancel_after_secs > 0.0:
                # Sleep for the specified time
                sleep_time = node.create_rate(cancel_after_secs)
                sleep_time.sleep()
                # Cancel the goal
                print("Cancelling goal")
                moveit2.cancel_execution()

            # Wait until the future is done
            while not future.done():
                rate.sleep()

            # Print the result
            print("Result status: " + str(future.result().status))
            print("Result error code: " + str(future.result().result.error_code))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)


if __name__ == "__main__":
    main()
