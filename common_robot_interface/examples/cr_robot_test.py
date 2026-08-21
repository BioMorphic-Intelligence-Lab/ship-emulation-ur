"""Simple test script for SyncRobot class using CR Controller.
Ethernet TCP/IPv4: 192.168.5.10, ip: 192.168.5.1
"""

import time
import numpy as np

from cri.robot import SyncRobot
from cri.controller import CRController as Controller

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)            # x->left, y->back, z->up, rz->anticlockwise
    work_frame = (0, -350, 200, -180, 0, 0)   # x->left, y->back, z->up, rz->anticlockwise

    with SyncRobot(Controller()) as robot:

        # Set TCP and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, -100, 0, 0, 0)
        robot.speed = 50

        # zero last joint
        robot.move_joints([*robot.joint_angles[:-1], 0])
        
        # Display robot info
        print("speed: ", robot.speed) # delayed until next run
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles and inital pose in work frame
        print("Initial joint angles in work frame: {}".format(robot.joint_angles))
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Target origin pose in work frame: {}".format(robot.target_pose))
        print("Origin pose in work frame: {}".format(robot.pose))
        
        # Increase and decrease all joint angles 
        print("Decreasing and increasing all joint angles ...")
        robot.move_joints(robot.joint_angles - (10,)*6)   
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        robot.move_joints(robot.joint_angles + (10,)*6)  
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))

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
        print("Roll right and left ...")        
        robot.move_linear((0, 0, 0, 30, 0, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))
        
        # Roll forwward and backward
        print("Roll forward and backward ...")        
        robot.move_linear((0, 0, 0, 0, 30, 0))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn clockwise and anticlockwise around work frame z-axis
        print("Turning clockwise and anticlockwise around work frame z-axis ...")        
        robot.move_linear((0, 0, 0, 0, 0, 30))
        print("Target pose in work frame: {}".format(robot.target_pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # # Make a circular move down/up, via a point on the right/left
        # print("Making a circular move down and up, via a point on the right/left ...")
        # robot.move_circular((0, 20, 20, 0, 0, 0), (0, 0, 40, 0, 0, 0))
        # robot.move_circular((0, -20, 20, 0, 0, 0), (0, 0, 0, 0, 0, 0))   

        # Move to offset pose then tap down and up in sensor frame
        print("Moving to 30 mm/ 10 deg offset in pose ...")         
        robot.move_linear((50, 50, 50, 30, 30, 30))
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        print("Pose in coord frame: {}".format(robot.pose))
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Pose in coord frame: {}".format(robot.pose))
        robot.move_linear((0, 0, -50, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Turn on servo mode and move through a sequence of waypoints
        robot.coord_frame = work_frame
        robot.move_linear((0, 0, 0, 0, 0, 0))
        
        print("Moving continuously through poses ...")         
        robot.controller.servo_mode = True
        robot.controller.servo_delay = 0.030 - 0.015
        # time_start = time.time()
        for a in np.arange(0, 2*np.pi, 2*np.pi/100):
            pose = (50*np.cos(a)-50, 50*np.sin(a), 0, 0, 0, 0)
            robot.move_linear(pose)
            # print(f"Time: {time.time()-time_start} Pose: {robot.pose}")
        robot.controller.servo_mode = False

        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.move_linear((0, 0, 0, 0, 0, 0))
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))

        # zero last joint
        robot.move_joints([*robot.joint_angles[:-1], 0])

if __name__ == '__main__':
    main()
