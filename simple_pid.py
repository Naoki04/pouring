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


from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController
from robotiqGripper import RobotiqGripper

import importlib
ftsensor = importlib.import_module("force-torque-sensor")
ATI_readings = ftsensor.ATI_readings
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
def ft_warmup(minutes=5, show=False):
    
    sec = minutes * 60

    # ft setup
    ati_fts = [ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360022506),
               ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360023125)]  # weight:360022506, in-whist:360023125))
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
    with tqdm(total=sec) as pbar:
        while time.time() - start_t < sec:
            ati_ft.get_weight()
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
    ati_ft_weight = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360022506) # weight:360022506, in-whist:360023125)
    ati_ft_inwhist = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360023125) # weight:360022506, in-whist:360023125)
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
        pouring_point = (50, 60, 30, 0, 0, 0)
        robot.move_linear(pouring_point) # 奥_手前, 右_左, 下_上


        """ 
        Pouring
        """
        realtime_plot1d.update(weight)
        robot.angular_speed = 1000
        robot.move_joints(robot.joint_angles + [0,0,0,0,0,-45])
        

        pouring_flag = True


        e = target_w - weight # Current error
        e1 = target_w - weight # Previous error
        ei = 0 # error before previous error

        
        try: 
            while(pouring_flag):
                realtime_plot1d.update(weight)
                # Get weight
                e = target_w - weight
                ei += e
                # Caliculate Error
                d_theta = Kp * e1 + Kd * (e-e1) + Ki * ei
                if d_theta >= angle_limit:
                    print("POSE LIMITATION")
                    print(d_theta)
                    break
                robot.move_joints(robot.joint_angles + [0,0,0,0,0,-d_theta])
                #controller.move_linear_velocity((0, 0, 0, -d_theta, 0, 0), 1000, 0.2)
                print(d_theta, robot.pose[3])
                realtime_plot1d.update(weight)
                
                e1 = e
                
                
                if weight >= target_w:
                    pouring_flag = False
                    print("Target Volume Achieved")
                if abs(robot.pose[3]) >= angle_limit:
                    pouring_flag = False
                    print("POSE LIMITATION")

        except KeyboardInterrupt:
             # Grasp Back
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
            
            
        
        robot.move_linear(pouring_point)
        robot.move_linear(pouring_point)
        robot.angular_speed = 10

        """ 
        Return to status quo
        """
        # Grasp Back
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
    #ft_warmup(minutes=2,show=2) #1h~2h(1.5h) # show=0:don't show, show=1:show weight, show=2:show inwhist
    main()