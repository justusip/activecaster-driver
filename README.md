# Active Caster Driver
A ROS driver to interface with the Active Caster Controller.

## Usage
1. Connect the computer to the controller via a USB cable 
2. Chmod or set udev rules to let `hidraw*` be accessible to the program
3. `roslaunch cheetah_driver start.launch`
4. The driver will automatically search for the device and connect to it, dont touch it otherwise it will be angry ah

### Setting RPM of VESC
```rostopic pub /cheetah/rpm std_msgs/Int32 [RPM]```
The publishing has to be ongoing, otherwise the controller will stop sending signal to VESC after 1 second of inactivity.


### Getting feedback from VESC
```rostopic echo /cheetah/[motor id]/vesc``` echoes custom message `VescStatus`
```
bool connected
float32 rpm
float32 current
float32 pos
float32 mosfetTemp
float32 motorTemp
```
`motor id` ranges from 0-3, which are VESC of front left, front right, rear left and rear right respectively. Feedback of all four VESC are received within the same USB packet and published simultaneously, every ~10ms.

### Setting angle of DJI Motors
TODO

### Getting angle from DJI Motors
TODO
