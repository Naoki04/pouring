# -*- coding: utf-8 -*-
"""Simple test script for AsyncRobot class using RTDEController.
"""

import time
import minimalmodbus as mm
import serial
import binascii
import numpy as np
from tqdm import tqdm
import threading
import csv

from circular_trajectory import positions as trajectory

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController
from robotiqGripper import RobotiqGripper
from matplotlib import pyplot as plt

import importlib
ftsensor = importlib.import_module("force-torque-sensor")
ATI_readings = ftsensor.ATI_readings
userAxis_FT35016 = ftsensor.userAxis_FT35016
userAxis_FT39618 = ftsensor.userAxis_FT39618

RealtimePlot1D = ftsensor.RealtimePlot1D

np.set_printoptions(precision=2, suppress=True)

"""
Parameters
"""
Kp = 0.005
Kd = 0.01
Ki = 0.0001

target_w =120
angle_limit = 70

g = 9.80665


# Variable to store weight
weight = 0

# Warm up process for FT sensor. 1.5h is recommended.
def ft_warmup(minutes=5, show=False, save=False):
    
    sec = minutes * 60
    if save:
        csv_path = "data/csv/" + time.strftime("%Y%m%d_%H%M%S") + "warmup.csv"

    # ft setup
    ati_fts = [ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360022506, userAxis=userAxis_FT39618),
               ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360023125, userAxis=userAxis_FT35016)]  # weight:360022506, in-whist:360023125))
    for ati_ft in ati_fts:
        print(ati_ft.__str__())
        ati_ft.calibration()  # output
    
    start_t = time.time()

    ati_ft = ati_fts[0]

    if show >= 1:
        # Graph
        x_tick = 1  # 時間方向の間隔
        length = sec*6  # プロットする配列の要素数
        realtime_plot1d = RealtimePlot1D(x_tick, length)
        ati_ft = ati_fts[show-1]
    
    previous_time = time.time()
    
    if save: 
        with open(csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "weight[z](g)", "in-whist[z](g)"])
            
            with tqdm(total=sec) as pbar:
                while time.time() - start_t < sec:
                    ati_fts[0].get_weight()
                    ati_fts[1].get_weight()
                    pbar.update(time.time() - previous_time)
                    previous_time = time.time()
                    if show >= 1:
                        weight = -ati_ft.forces[2] / g * 1000  # g
                        
                        realtime_plot1d.update(weight)
                        writer.writerow([time.time()-start_t, -ati_fts[0].forces[2] / g * 1000, -ati_fts[1].forces[2] / g * 1000])
        
    else:
        with tqdm(total=sec) as pbar:
            while time.time() - start_t < sec:
                ati_fts[0].get_weight()
                ati_fts[1].get_weight()
                pbar.update(time.time() - previous_time)
                previous_time = time.time()
                if show >= 1:
                    weight = -ati_ft.forces[2] / g * 1000  # g
                    realtime_plot1d.update(weight)


# get weight and save it into CSV file on subprocess
def get_weight(fts):
    print("Subprocess started.")
    global weight 

    csv_path = "data/csv/" + time.strftime("%Y%m%d_%H%M%S") + ".csv"
    

    with open(csv_path, 'w', newline='') as file:
        writer = csv.writer(file)
        #print(["timestamp", str(fts[0].daq_device.configU6()["SerialNumber"])+"(g)", str(fts[1].daq_device.configU6()["SerialNumber"])+"(g)"])
        writer.writerow(["timestamp", str(fts[0].daq_device.configU6()["SerialNumber"])+"(g)", str(fts[1].daq_device.configU6()["SerialNumber"])+"(g)"])
        while True:
            fts[0].get_weight()
            fts[1].get_weight()
            weight = -fts[0].forces[2] / g * 1000  # g
            writer.writerow([time.time(), fts[0].forces/g*1000, fts[1].forces/g*1000])
            
        #realtime_plot1d.update(weight)
        #print(weight)

def main():
    # FT Setup
    ati_ft_weight = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360022506, userAxis=userAxis_FT39618) # weight:360022506, in-whist:360023125)
    ati_ft_inwhist = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360023125, userAxis=userAxis_FT35016) # weight:360022506, in-whist:360023125)
    fts = [ati_ft_weight, ati_ft_inwhist]
    for ati_ft in fts:
        print("Checking F/T sensor:", ati_ft.daq_device.configU6()["SerialNumber"])
        print(ati_ft.__str__())
        ati_ft.calibration()  # output

    # Graph
    x_tick = 1  # 時間方向の間隔
    length = 100  # プロットする配列の要素数
    realtime_plot1d = RealtimePlot1D(x_tick, length)

    ft_thread = threading.Thread(target=get_weight, args=(fts,))
    ft_thread.setDaemon(True)
    ft_thread.start()

    global weight

    # Robot setup
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (800, -345, 302, 180, 0, -90)     # base frame: x->right, y->back, z->up # Corner of the box
    #work_frame = (487.0, -109.1, 341.3, 180, 0, 180)    # base frame: x->front, y->right, z->up

    # Gripper setup
    instrument = mm.Instrument('/dev/tty.usbserial-DA6UJVT6', 9, debug = True)
    # instrument.serial.port                     # this is the serial port name
    instrument.serial.baudrate = 115200   

    grip=RobotiqGripper("/dev/tty.usbserial-DA6UJVT6",slaveaddress=9)
    grip.reset()
    grip.printInfo()

    
    print(f'z_weight {weight}g')
    time.sleep(1)

    
    with AsyncRobot(SyncRobot(RTDEController(ip='192.168.1.141'))) as robot:
    # For testing in URSim simulator
#    with AsyncRobot(SyncRobot(RTDEController(ip='127.0.0.1'))) as robot:
        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.tcp = (0, 330, 0, 90, 180, 0)
        robot.linear_speed = 100
        robot.angular_speed = 10
        robot.coord_frame = work_frame
        controller = robot.sync_robot.controller
      
        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        """ 
        Movement before Pouring 
        """
        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))# wait until end of movement
        grip.activate()
        grip.printInfo()

        # Cup Grasping
        grip.goTo(90)
        robot.move_linear((0, -160, 0, 0, 0, 0))
        grip.goTo(160)
        # Grasp up
        robot.move_linear((0, -160, -100, 0, 0, 0))

        # Pouring Point
        pouring_point = (70, 60, 30, 0, 0, 0)#(50, 60, 30, 0, 0, 0)
        robot.move_linear(pouring_point) # 奥_手前, 右_左, 下_上


        """ 
        Pouring
        """
        realtime_plot1d.update(weight)
        robot.angular_speed = 1000
        initial_theta = 45
        robot.move_linear((pouring_point[0], pouring_point[1], pouring_point[2], 0, +initial_theta, 0))
        

        pouring_flag = True
        pour_started_flag = False

        e = target_w - weight # Current error
        e1 = target_w - weight # Previous error
        ei = 0 # error before previous error

        robot.blend_radius = 5
        global trajectory
        trajectory_index = 0
        theta = initial_theta
        robot.linear_speed = 50
        real_path = []
        
        pre_weight = weight
        try: 
            while(pouring_flag):
                realtime_plot1d.update(weight) #weight
                pre_weight = weight
                # Get weight
                e = target_w - weight
                ei += e
                # Caliculate Error
                d_theta = Kp * e1 + Kd * (e-e1) + Ki * ei
                if d_theta >= angle_limit:
                    print("POSE LIMITATION")
                    print(d_theta)
                    break
                target = trajectory[trajectory_index]
                
                theta += d_theta

                if weight >= 5:
                    pour_started_flag = True

                if pour_started_flag == False:
                    target = [0, 0]
                    trajectory_index -= 1

                target[0] += pouring_point[0]
                target[1] += pouring_point[1]
                
                print("goal", (target[0], target[1], pouring_point[2], 0, theta, 0))
                
                robot.move_linear((target[0], target[1], pouring_point[2], 0, theta, 0))
                
                print("theta", theta, "real", robot.pose[4])
                realtime_plot1d.update(weight)

                trajectory_index += 1
                real_path.append([robot.pose[0], robot.pose[1]])
                
                e1 = e

                print("---")
                
                
                if weight >= target_w:
                    pouring_flag = False
                    print("Target Volume Achieved")
                if abs(robot.pose[3]) >= angle_limit:
                    pouring_flag = False
                    print("POSE LIMITATION")
                if trajectory_index == trajectory.shape[0]-1:
                    pouring_flag = False
                    print("End of Trajectory")
                    

        except KeyboardInterrupt:
             # Grasp Back
            robot.blend_radius = 0
            robot.angular_speed = 10
            robot.move_linear(pouring_point)
            robot.move_linear((80, 70, 0, 0, 0, 0))
            robot.move_linear((0, -160, -100, 0, 0, 0))
            robot.linear_speed = 20
            robot.move_linear((0, -160, 0, 0, 0, 0))
            robot.move_linear((0, -160, 0, 0, 0, 0))
            robot.linear_speed = 100
            grip.goTo(90)
            robot.move_linear((0, 0, 0, 0, 0, 0))

            return KeyboardInterrupt
        
        print(real_path)
        real_x = [path[0] for path in real_path]
        real_y = [path[1] for path in real_path]
        plt.plot(real_x, real_y)
        plt.show()
            
            
        
        robot.move_linear(pouring_point)
        robot.move_linear(pouring_point)
        robot.angular_speed = 10

        """ 
        Return to status quo
        """
        # Grasp Back
        robot.blend_radius = 0
        robot.move_linear((0, -160, -100, 0, 0, 0))
        robot.linear_speed = 20
        robot.move_linear((0, -160, 0, 0, 0, 0))
        robot.move_linear((0, -160, 0, 0, 0, 0))
        robot.linear_speed = 100
        grip.goTo(90)
        robot.move_linear((0, 0, 0, 0, 0, 0))
        time.sleep(100)
        return 0
    
        

     
if __name__ == '__main__':
    #ft_warmup(minutes=180,show=2,save=True) #1h~2h(1.5h) # show=0:don't show, show=1:show weight, show=2:show inwhist
    main()