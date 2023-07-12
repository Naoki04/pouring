# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using RTDEController.
"""

import time
import threading

import numpy as np
import matplotlib.pyplot as plt

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController

from circular_trajectory import positions as trajectory

np.set_printoptions(precision=2, suppress=True)


def main():
    work_frame = (500, -345, 302, -90, 0, -90)  # base frame: x->right, y->back, z->up
    #     work_frame = (487.0, -109.1, 341.3, 180, 0, 180)    # base frame: x->front, y->right, z->up

    with AsyncRobot(SyncRobot(RTDEController(ip='192.168.1.141'))) as robot:
        # For testing in URSim simulator
        #    with AsyncRobot(SyncRobot(RTDEController(ip='127.0.0.1'))) as robot:
        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.tcp = (0, 0, 0, 0, 0, 0)
        robot.linear_speed = 50
        robot.angular_speed = 5
        robot.coord_frame = work_frame
        controller = robot.sync_robot.controller

        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        # Single velocity move
        #print("Making single velocity move, then stopping ...")
        controller.move_linear_velocity((10, 0, 0, 0, 0, 0), 20,3)
        print("aaa")
        controller.move_linear_velocity((0, 10, 0, 0, 0, 0), 20,3)
        print("bbb")
        controller.move_linear_velocity((0, 0, 10, 0, 0, 0), 20,3)
        print("ccc")
        
        #controller.move_linear_velocity((0, 0, 10, 0, 0, 0), 20, 10)
        #controller.move_linear_velocity((0, 0, 0, 2, 0, 0), 100,1)
        #controller.move_linear_velocity((0, 0, 0, 3, 0, 0), 100,1)
        #print("aaa")
        #controller.move_linear_velocity((0, 0, 0, -3, 0, 0), 100,1)
        
        #time.sleep(3)
        
        #controller.move_linear_velocity((0, 0, 0, 0, 0, 0), -10)
        #controller.stop_joints_velocity(100)
        #controller.stop_linear_velocity(1)
        time.sleep(100000)

        # Circular motion
        v = 10
        min_distance = 10
        
        print("Moving to starting point of circular motion ...")
        global trajectory
        trajectory = trajectory.copy()

        robot.move_linear((trajectory[0][0], trajectory[0][1], 0, 0, 0, 0))
        trajectory = np.delete(trajectory, 0, axis=0)
        time.sleep(1)
        


        real_trajectory = []
        targets = []

        pre_vel = np.array([0,0])
        duration = 0
        target = trajectory[0]

        while True:
            # 現在位置の取得
            pose = robot.pose
            real_trajectory.append([pose[0], pose[1]])
            print("position:", [pose[0], pose[1]])
            x0 = pose[0]
            y0 = pose[1]
        
            # 次のターゲットの選択
            target = np.array([x0,y0])
            distance = 0
            while( distance < min_distance):
                target = trajectory[0]
                print("target:", target)
                distance = ((target[0]-x0)**2 + (target[1]-y0)**2)**0.5
                trajectory = np.delete(trajectory, 0, axis=0)
                print("distance:", distance)
            targets.append(target)

            # 現在地から次のターゲットへのベクトル
            direction = np.array([target[0]-x0, target[1]-y0])

            L = 1/2 * np.linalg.norm(direction,ord=2)**2 * np.linalg.norm(pre_vel,ord=2) / np.dot(pre_vel, direction.T)

            
            # 移動時間と速度の計算
            if pre_vel[0] == 0 and pre_vel[1] == 0:
                duration = (distance/10)**0.5 # y = 1/2 at^2
                pre_vel = direction / np.linalg.norm(direction,ord=2) * v
            else:
                duration = 2*L/v
                dir = direction - L*pre_vel / np.linalg.norm(pre_vel,ord=2)
                pre_vel = dir / np.linalg.norm(dir,ord=2) * v
            print("duration:",duration)                        
            print("vel:", pre_vel)
           
            # 速度の入力
            controller.move_linear_velocity((pre_vel[0], pre_vel[1], 0, 0, 0, 0), 10, duration)

            if trajectory.shape[0] <= 10:
                print("Done!")
                break
            print("----")

        plt.plot([x[0] for x in real_trajectory], [x[1] for x in real_trajectory])
        plt.scatter([x[0] for x in targets], [x[1] for x in targets])
        plt.show()



        print("stop")
        time.sleep(10000)

        # Sequence of velocity moves
        print("Making multiple velocity moves, then stopping ...")
        controller.move_linear_velocity((40, 0, 0, 10, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.move_linear_velocity((0, 40, 0, 20, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.move_linear_velocity((0, 0, 40, -10, 0, 0), 20, 3)
        print("Calculating next move ...")
        time.sleep(2)
        controller.stop_linear_velocity(10)

        print("Waiting for 5 secs ...")
        time.sleep(5)

        print("Making multiple velocity moves, then stopping (multi-threaded) ...")

        # Sequence of velocity moves on background thread
        running = False
        lock = threading.Lock()
        velocity = (0, 0, 0, 0, 0, 0)
        accel = 10
        ret_time = 1/30

        def worker():
            while running:
                with lock:
                    worker_velocity = velocity
                    worker_accel = accel
                    worker_ret_time = ret_time
                controller.move_linear_velocity(worker_velocity, worker_accel, worker_ret_time)

        thread = threading.Thread(target=worker, args=[], kwargs={})
        running = True
        thread.start()

        try:
            with lock:
                velocity = (-40, 0, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, -40, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, 0, -40, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)
            with lock:
                velocity = (0, 0, 0, 0, 0, 0)
            print("Calculating next move ...")
            time.sleep(2)

        finally:
            running = False
            thread.join()

        print("Waiting for 5 secs ...")
        time.sleep(5)

        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))

        print("Final target pose in work frame: {}".format(robot.target_pose))
        print("Final pose in work frame: {}".format(robot.pose))


if __name__ == '__main__':
    main()
