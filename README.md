# pololu_ros
This is a ROS driver for the Pololu Simple Motor Controller. It uses the Pololu
Protocol to communicate over serial to control a motor.


## pololu_node

### Subscribed Topics

`/arm/vel` (std_msgs/Float32)<br>
Command to control the arm linear actuators.  Expects -1.0 to 1.0

`/bucket/vel` (std_msgs/Float32)<br>
Command to control the bucket linear actuators. Expects -1.0 to 1.0

`/sled/vel` (std_msgs/Float32)<br>
Command to control the pinion for the rack and pinion. Expects -1.0 to 1.0


### Published Topics
`/pololu/limit_switch` ([pololu_ros/LimitSwitch](https://github.com/jrobe/robotic-mining/blob/Kinetic/urmpspace/src/pololu_ros/msg/LimitSwitch.msg))<br>
This is a custom message type of two booleans, front and rear, that represent
if the limit switches are triggered.





## Package overview

The low level driver for this package is in the `src` directory.  It is setup
for a Daisy chain configuration, so all motors share the same serial connection
and all commands are sent with the device number and motors ignore commands
not directed to them.

The ROS specific code for this package is in the `nodes` directory. This
starts a ROS node and subscribes to topics to control the motors using the
Daisy chain driver.
