# Hop

A wheeled jumping robot for controls and character development

## Feature Demos

**Stair Climb**

**Terrain Navigation**

**Fall Recovery**

**Personality Interaction**

## Robot Build

**BOM**

[Bill of Materials Spreadsheet](https://docs.google.com/spreadsheets/d/1hSjXVyd_R7ku4igwh2TApJvZMaHaLdp0xNJVG1NKjF0/edit?usp=sharing)

**CAD**

[Onshape Model](https://cad.onshape.com/documents/d1510b7142eb756e40f1872d/w/890ef46705dcc90d9613f34d/e/74d313fd2036cc921f86eb4a)

**Electrical**

![Schematic Image](https://github.com/kghite/hop/blob/master/docs/hop_schematic.png?raw=true)

## Software System

**System Graph**

**FSM Flow**

**Controls Modules**

## Setup Instructions

**Onboard Rpi**

* [Install ROS Melodic for RPi Zero W](https://labs.fpv.umb.sk/ros/)
* Clone the `rpi` branch of the `hop` repo into `catkin_ws/src`
* Enable running the startup script on boot
  * `sudo vi /etc/rc.local` 
  * Add `sudo bash /home/<user>/catkin_ws/src/hop/rpi-boot.sh &` before the `exit 0` line