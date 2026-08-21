"""Simple test script for AsyncRobot class using SimController.
"""
import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import SimController
from cri.sim.utils.sim_utils import setup_pybullet_env

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)         # base frame: x->front, y->right, z->up
    work_frame = (350, 0, 100, -180, 0, 0)  # x->back,  y->left,  z->down, rz->clockwise
    
    embodiment = setup_pybullet_env()

    with AsyncRobot(SyncRobot(SimController(embodiment.arm))) as robot:

        # Set TCP and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, 0, 0, 0, 0)

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Increase and decrease all joint angles (async)
        print("Increasing and decreasing all joint angles ...")
        robot.async_move_joints(robot.joint_angles + (10,)*6)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.async_move_joints(robot.joint_angles - (10,)*6)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))

        # Move backward and forward (async)
        print("Moving backward and forward (async) ...")
        robot.async_move_linear((50, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move right and left
        print("Moving right and left (async) ...")
        robot.async_move_linear((0, 50, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move down and up (async)
        print("Moving down and up (async) ...")
        robot.async_move_linear((0, 0, 50, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Roll right and left (async)
        print("Rolling right and left (async) ...")
        robot.async_move_linear((0, 0, 0, 30, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Roll forward and backward (async)
        print("Rolling forward and backward (async) ...")
        robot.async_move_linear((0, 0, 0, 0, 30, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Turn clockwise and anticlockwise around work frame z-axis (async)
        print("Turning clockwise and anticlockwise around work frame z-axis (async) ...")
        robot.async_move_linear((0, 0, 0, 0, 0, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move to offset pose then tap down and up in sensor frame (async)
        print("Moving to 30 mm / 10 deg offset in pose (async) ...")
        robot.async_move_linear((30, 30, 30, 10, 10, 10))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up (async) ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        robot.async_move_linear((0, 0, -30, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        print("Moving to origin of work frame ...")
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
