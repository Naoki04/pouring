"""
theta_minに動く
while theta < theta_max:
    # thetaの取得
    # Rawdataの取得
    # biasの保存
    # thetaの更新
"""
import csv
import time
import minimalmodbus as mm
import numpy as np
from tqdm import tqdm

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController
from robotiqGripper import RobotiqGripper

from bias import read_bias_table, find_bias

import importlib
ftsensor = importlib.import_module("force-torque-sensor")
ATI_readings = ftsensor.ATI_readings
userAxis_FT35016 = ftsensor.userAxis_FT35016

RealtimePlot1D = ftsensor.RealtimePlot1D

theta_min = -90
theta_max = 90
gripper_pos = 0
g = 9.80665

csv_file = "data/csv/save/20230518_152715inwhist_calibration.csv"


def main():

    # Bias table
    bias_table = read_bias_table(csv_file)

    # Inwhist FTsensor setup
    ati_ft = ATI_readings(resolutionIndex=1, gainIndex=0, settlingFactor=0, differential=True, serial=360023125, userAxis=userAxis_FT35016) # weight:360022506, in-whist:360023125)
    print("Checking F/T sensor:", ati_ft.daq_device.configU6()["SerialNumber"])
    print(ati_ft.__str__())

    # Graph
    x_tick = 1  # 時間方向の間隔
    length = 200  # プロットする配列の要素数
    realtime_plot1d = RealtimePlot1D(x_tick, length)

    # Robot setup
    base_frame = (0, 0, 0, 0, 0, 0)
    work_frame = (800, -345, 302, 180, 0, -90)     # base frame: x->right, y->back, z->up # Corner of the box

    # Gripper setup
    instrument = mm.Instrument('/dev/tty.usbserial-DA6UJVT6', 9, debug = True)
    # instrument.serial.port                     # this is the serial port name
    instrument.serial.baudrate = 115200   
    grip=RobotiqGripper("/dev/tty.usbserial-DA6UJVT6",slaveaddress=9)
    grip.reset()
    grip.printInfo()
  

    with AsyncRobot(SyncRobot(RTDEController(ip='192.168.1.141'))) as robot:
    # For testing in URSim simulator
#    with AsyncRobot(SyncRobot(RTDEController(ip='127.0.0.1'))) as robot:
        # Set TCP, linear speed,  angular speed and coordinate frame
        robot.tcp = (0, 330, 0, 90, 180, 0)
        robot.linear_speed = 100
        robot.angular_speed = 10
        robot.coord_frame = work_frame
        
      
        # Display robot info
        print("Robot info: {}".format(robot.info))

        # Display initial joint angles
        print("Initial joint angles: {}".format(robot.joint_angles))

        # Display initial pose in work frame
        print("Initial pose in work frame: {}".format(robot.pose))

        """ 
        Move to work position
        """
        # Move to origin of work frame
        print("Moving to origin of work frame ...")
        robot.move_linear((0, 0, 0, 0, 0, 0))
        robot.move_linear((0, 0, 0, 0, 0, 0))# wait until end of movement

        grip.activate()
        grip.printInfo()
        print("SET OBJECT!!")
        time.sleep(3)
        grip.goTo(gripper_pos)

        ati_ft.calibration()  # output

        
        """
        Measure
        """
        theta = theta_min

        # angle at start pose
        original_angle = robot.joint_angles
        target = original_angle.copy() - [0,0,0,0,0,theta_min]
        robot.move_joints(target)

        
        with tqdm(total=theta_max-theta_min) as pbar:

            while theta < theta_max:
                time.sleep(3)
                # thetaの取得
                real_theta = robot.joint_angles[5]
                
                # biasの呼び出し
                bias = find_bias(bias_table, real_theta)

                # biasの更新
                ati_ft.bias = bias
                print(real_theta, ati_ft.bias)

                # 重量計算
                ati_ft.get_weight()
                res_weight = (ati_ft.forces[0]**2 + ati_ft.forces[1]**2 + ati_ft.forces[2]**2)**(1/2) /g *1000
                realtime_plot1d.update(res_weight)
                
                # thetaの更新
                theta += 1
                target = original_angle.copy() - [0,0,0,0,0,theta]
                robot.move_joints(target)
                robot.move_joints(target)

                pbar.update(1)
                

if __name__ == "__main__":
    main()