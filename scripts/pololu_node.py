#!/usr/bin/python
from __future__ import division
from pololu_driver import clip, Daisy # serial controller for controller in Daisy chain

import rospy
import threading
from std_msgs.msg import Float32, Header, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pololu_ros.msg import LimitSwitch

# TODO: if necessary, add more try and excepts for error catching

class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node") 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to pololu daisy chain")
        self.last_set_speed_time = rospy.get_rostime()
        self.port = rospy.get_param("~port")

        #self.timeout = rospy.get_param("~timeout")  # time between hearing commands before we shut off the motors

        self.timeout = 1.5
        rospy.loginfo("Daisy chain port: %s", self.port)
        rospy.loginfo("Daisy node timeout = %s", self.timeout)

        # get device numbers from ros parameter server (see launch~_node.launch)
        self.dump_spin_left_devnum = rospy.get_param("~dump_spin_left")
        self.dump_spin_right_devnum = rospy.get_param("~dump_spin_right")
        self.extend_linear_left_devnum = rospy.get_param("~extend_linear_left")
        self.extend_linear_right_devnum = rospy.get_param("~extend_linear_right")
        self.insert_linear_devnum = rospy.get_param("~insert_linear")

        # get device numbers from ros parameter server (see launch~_node.launch)
        self.dump_spin_left = Daisy(self.dump_spin_left_devnum, port=self.port) # 1st one sets port
        self.dump_spin_right = Daisy(self.dump_spin_right_devnum)
        self.extend_linear_left = Daisy(self.extend_linear_left_devnum)
        self.extend_linear_right = Daisy(self.extend_linear_right_devnum)
        self.insert_linear = Daisy(self.insert_linear_devnum)

        # TODO (p1): add some checks to make sure that devices boot up 

        rospy.sleep(0.1) # wait for params to get set

        # Subscribers
        self.insert_sub = rospy.Subscriber("insert_linear", Float32, self.insert_callback, queue_size=1)
        self.extend_sub = rospy.Subscriber("extend_linear", Float32, self.extend_callback, queue_size=1)
        self.dump_sub = rospy.Subscriber("dump_spin", Float32, self.dump_callback, queue_size=1)
        #self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        #rospy.Timer(rospy.Duration(0.1), self.publish_limits)

        self._has_showed_message = False  # flag to indicate we have showed the motor shutdown message

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        self.dump_spin_left.stop()

    def run(self):
        """Run the main ros loop"""
        rospy.loginfo("Starting Daisy node loop")
        r_time = rospy.Rate(10) #10 Hz looping

        while not rospy.is_shutdown():
            # TODO: consider making individual checks for each of the controllers,
            # so we don't stop everything when one stops receiving
            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > self.timeout:
                self._stop_all_motors()
                if not self._has_showed_message:
                    rospy.loginfo("No commands received in the last %d seconds. Shutting down motors",self.timeout)
                self._has_showed_message = True
            else:
                self._has_showed_message = False
            r_time.sleep()


    def insert_callback(self, cmd):
        rospy.logdebug("Velocity cmd to insert linear actuator: %.4f", cmd.data)

        motor_cmd = clip(int(cmd.data * 3200), 0, 3200)
        self.last_insert_msg_time = rospy.get_rostime()

        rospy.logdebug("Insert linear actuator serial cmd = %d", motor_cmd)

        self.insert.drive(motor_command)

        ### drive both linear actuators with the same cmd
        ##self.sled_left.drive(-motor_command)
        ##self.sled_right.drive(motor_command)

    def extend_sub(self, cmd):
        rospy.logdebug("Velocity cmd to extend linear actuators: %.4f", cmd.data)
        pass

    def dump_sub(self, cmd):
        rospy.logdebug("Velocity cmd to dump spin motors: %.4f", cmd.data)
        pass


    def shutdown(self):
        """Handle shutting down the node"""
        rospy.loginfo("Shutting down daisy node")

        # so these don't get called while the node is shutting down
        # (need to add checks in case it immediately shuts down. (i think))
        if hasattr(self, "insert_sub"):
            self.insert_sub.unregister()
        if hasattr(self, "extend_sub"):
            self.extend_sub.unregister()
        if hasattr(self, "dump_sub"):
            self.dump_sub.unregister()
        self._stop_all_motors()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting daisy node")


##    def publish_diagnostics(self, event):
##        """Generate a diagnostic message of device statuses, voltage levels,
##        and limit switch triggers. Not critical code.
##        """
##        with self.lock:
##            diagnostic_arr = DiagnosticArray()  # create object to hold data to publish
##            # Generate a ROS message header for time stamping
##            header = Header()
##            header.stamp = rospy.Time.now()
##            diagnostic_arr.header = header
##
##            # For all the devices, set the status based on Pololu serial variables
##            # and append these statues to the diagnostic_arr status field
##            for i in range(len(self.devices)):
##                dev = self.devices[i]
##                name = self.dev_names[i]
##                status = DiagnosticStatus()
##                status.name = name
##                status.hardware_id = str(self.devices[i].dev_num)
##
##                errstat = dev.get_error_status()
##                sererr = dev.get_serial_errors()
##                if errstat == "OK" and sererr == "OK":
##                    status.level = 0
##                    status.message = errstat + "\n" + sererr
##                else:
##                    status.level = 1
##                    status.message = errstat + "\n" + sererr
##
##                # Get voltage level and set error status if it is too high or low
##                voltage_level = dev.get_input_voltage()
##                if voltage_level > 30 or voltage_level < 20:
##                    status.level = 2
##                    status.message = "Voltage out of range: %.2f" % (voltage_level)
##
##                status.values.append(KeyValue("Voltage", "%.2f" % voltage_level))
##                status.values.append(KeyValue("Speed", str(dev.get_speed())))
##
##                diagnostic_arr.status.append(status)
##
##            self.diagnostic_pub.publish(diagnostic_arr)

