"""Simple test script for AsyncRobot class using SimController.
"""
import numpy as np

from cri.robot import SyncRobot
from cri.controller import SimController
from cri.sim.utils.sim_utils import setup_pybullet_env


np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)         # x->front, y->right, z->up,   rz->anticlockwise
    work_frame = (350, 0, 100, -180, 0, 0)  # x->back,  y->left,  z->down, rz->clockwise

    embodiment = setup_pybullet_env()
    with SyncRobot(SimController(embodiment.arm)) as robot:

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
        print("Target origin pose in work frame: {}".format(robot.target_pose))
        print("Origin pose in work frame: {}".format(robot.pose))

        # Increase and decrease all joint angles
        print("Increasing and decreasing all joint angles ...")
        robot.move_joints(robot.joint_angles + (10,)*6)
        print("Target joint angles after increase: {}".format(robot.target_joint_angles))
        print("Joint angles after increase: {}".format(robot.joint_angles))
        robot.move_joints(robot.joint_angles - (10,)*6)
        print("Target joint angles after decrease: {}".format(robot.target_joint_angles))
        print("Joint angles after decrease: {}".format(robot.joint_angles))

        # Move backward and forward
        print("Moving backward and forward ...")
        robot.move_linear((50, 0, 0, 0, 0, 0)) 
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move right and left
        print("Moving right and left ...")
        robot.move_linear((0, 50, 0, 0, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move down and up
        print("Moving down and up ...")
        robot.move_linear((0, 0, 50, 0, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll right and left
        print("Rolling right and left ...")
        robot.move_linear((0, 0, 0, 30, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Roll forward and backward
        print("Rolling forward and backward ...")
        robot.move_linear((0, 0, 0, 0, 30, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")
        robot.move_linear((0, 0, 0, 0, 0, 30))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Move to offset pose then tap down and up in sensor frame
        print("Moving to 30 mm/ 10 deg offset in pose ...")
        robot.move_linear((30, 30, 30, 10, 10, 10))
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up ...")
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        print("Pose in coord frame: {}".format(robot.pose))
        print("Target pose in coord frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Pose in coord frame: {}".format(robot.pose))
        robot.move_linear((0, 0, -30, 0, 0, 0))
        print("Pose in coord frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
