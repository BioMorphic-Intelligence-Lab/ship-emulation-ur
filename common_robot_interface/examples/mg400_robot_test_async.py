"""Simple test script for AsyncRobot class using MG400 Controller.
Ethernet TCP/IPv4: 192.168.1.10, ip: 192.168.1.6
"""

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import MG400Controller as Controller

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)  # base frame: x->front, y->left, z->up, rz->anticlockwise
    work_frame = (300, 0, 0, -180, 0, 0) 

    with AsyncRobot(SyncRobot(Controller())) as robot:

        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, -50, 0, 0, 0)    # (0, -38, 0, 0, 0, 0) # right angle mount
        robot.speed = 50
        
        # zero last joint
        # robot.move_joints([*robot.joint_angles[:-1], 0])

        # Display robot info
        print("speed: ", robot.speed) # delayed until next run
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles and inital pose in work frame
        print("Initial joint angles: {}".format(robot.joint_angles))
        print("Initial pose in work frame: {}".format(robot.pose))
        
        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        
        # print("Robot joint angles",robot.joint_angles)
        print("Robot pose: {}".format(robot.pose))

        # Increase and decrease all joint angles (async)
        print("Increasing and decreasing all joint angles ...")
        robot.async_move_joints(robot.joint_angles + (10,)*4)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        robot.async_move_joints(robot.joint_angles - (10,)*4)
        print("Getting on with something else while command completes ...")      
        robot.async_result()
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        
        # Move backward and forward (async)
        print("Moving backward and forward (async) ...")  
        robot.async_move_linear((50, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")      
        robot.async_result()
        
        # Move right and left (async)
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

        # Turn clockwise and anticlockwise around work frame z-axis (async)
        print("Turning clockwise and anticlockwise around work frame z-axis (async) ...")   
        robot.async_move_linear((0, 0, 0, 0, 0, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Move to offset pose then tap down and up in sensor frame (async)
        print("Moving to 50 mm / 30 deg offset in pose (async) ...")
        robot.async_move_linear((50, 50, 50, 0, 0, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up (async) ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        robot.async_move_linear((0, 0, -50, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.coord_frame = work_frame
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()

        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))

        # zero last joint
        robot.move_joints([*robot.joint_angles[:-1], 0])
        
if __name__ == '__main__':
    main()

