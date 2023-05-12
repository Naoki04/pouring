# FTDI driver for arm64 mac/linux is not supported.
-> Use intel Linux computer instead.

# To Start
- Connect dock before launching Ubuntu VM. (Network will be connected to mac)
- Launch VM.
- Plug off and on USBs on dock, and select VM to be connected.

or

- Launch VM
- Plug off usbs on dock
- Connect dock to laptop
- Select mac to be connected
- Plug in usbs
- Select VM to be connected

# FT sensor
-> Using LabJack Python
https://github.com/labjack/LabJackPython

# Robo Control
-> Using CRI
https://github.com/nlepora/cri

How to install script file to UR robot.
https://robodk.com/doc/en/Robots-Universal-Robots-How-load-Script-file.html

# Gripper Control
-> minimalmodbus
https://github.com/castetsb/pyRobotiqGripper

-> No answear error is solved by minimalmodbus setting: https://dof.robotiq.com/discussion/92/controlling-the-robotiq-2-finger-gripper-with-modbus-commands-in-python?_ga=2.17431289.1133307955.1542044765-1547192274.1535646624

---
# Installation
## CRI
```
$ cd cri
$ pip3 install -e .
$ python setup.py install
```

## pyRobotiqGripper
```
$ pip3 install minimalmodbus
```

## LabJackPython
-> a bit complex.

