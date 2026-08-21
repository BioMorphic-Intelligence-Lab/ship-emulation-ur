"""Simple test script for AsyncRobot class using CR Controller.
Ethernet TCP/IPv4: 192.168.5.10, ip: 192.168.5.1
"""

import numpy as np

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import CRController as Controller

np.set_printoptions(precision=2, suppress=True)


def main():
    base_frame = (0, 0, 0, 0, 0, 0)  # base frame: x->front, y->left, z->up, rz->anticlockwise
    work_frame = (0, -300, 200, -180, 0, 0)  # base frame: x->front, y->left, z->up, rz->anticlockwise

    with AsyncRobot(SyncRobot(Controller(ip='192.168.5.1'))) as robot:

        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.coord_frame = work_frame
        robot.tcp = (0, 0, -100, 0, 0, 0)
        robot.speed = 20
        
        # Display robot info
        print("speed: ", robot.speed)
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles and inital pose in work frame
        print("Initial joint angles: {}".format(robot.joint_angles))
        print("Initial pose in work frame: {}".format(robot.pose))
        
        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Increase and decrease all joint angles (async)
        print("Increasing and decreasing all joint angles ...")
        robot.async_move_joints(robot.joint_angles + (5,)*6)
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target joint angles: {}".format(robot.target_joint_angles))
        print("Joint angles: {}".format(robot.joint_angles))
        robot.async_move_joints(robot.joint_angles - (5,)*6)
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
        
        # Roll right and left (async)
        print("Roll right and left (async) ...")
        robot.async_move_linear((0, 0, 0, 30, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        
        # Roll forward and backward (async)
        print("Roll forward and backward (async) ...")
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

        # # Make a circular move down/up, via a point on the right/left
        # print("Making a circular move down and up, via a point on the right/left ...")
        # robot.async_move_circular((0, 20, 20, 0, 0, 0), (0, 0, 40, 0, 0, 0))
        # print("Getting on with something else while command completes ...")
        # robot.async_result()
        # robot.async_move_circular((0, -20, 20, 0, 0, 0), (0, 0, 0, 0, 0, 0))   
        # print("Getting on with something else while command completes ...")
        # robot.async_result()

        # Move to offset pose then tap down and up in sensor frame (async)
        print("Moving to 50 mm / 30 deg offset in pose (async) ...")
        robot.async_move_linear((50, 50, 50, 30, 30, 30))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Target pose after offset move: {}".format(robot.target_pose))
        print("Pose after offset move: {}".format(robot.pose))
        print("Tapping down and up (async) ...")
        robot.coord_frame = base_frame
        robot.coord_frame = robot.target_pose
        robot.async_move_linear((0, 0, 50, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        robot.coord_frame = work_frame
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
  
        # Turn on servo mode and move through a sequence of waypoints
        print("Moving continuously through poses ...")         
        robot.sync_robot.controller.servo_mode = True
        robot.sync_robot.controller.servo_delay = 0.030 - 0.015
        for a in np.arange(0, 2*np.pi, 2*np.pi/100):
            pose = (50*np.cos(a)-50, 50*np.sin(a), 0, 0, 0, 0)
            robot.async_move_linear(pose)
            robot.async_result()
        robot.sync_robot.controller.servo_mode = False
        
        # Finish
        print("Moving to origin of work frame ...")
        robot.coord_frame = work_frame
        robot.async_move_linear((0, 0, 0, 0, 0, 0))
        print("Getting on with something else while command completes ...")
        robot.async_result()
        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
