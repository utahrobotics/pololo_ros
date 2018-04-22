#!/usr/bin/python
from __future__ import division
from pololu_driver import Daisy # serial controller for controller in Daisy chain

import rospy
import threading
from std_msgs.msg import Float32, Header, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pololu_ros.msg import LimitSwitch

# TODO: if necessary, add more try and excepts for error catching


class Node(object):
    def __init__(self):
        """Init ros node"""
        self.lock = threading.Lock()
        rospy.init_node("pololu_node") #TODO: remove 2nd param when done debugging
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to pololu daisy chain")
        self.last_set_speed_time = rospy.get_rostime()
        self.port = rospy.get_param("~port")

        #self.timeout = rospy.get_param("~timeout")  # time between hearing commands before we shut off the motors

        self.timeout = 1.5
        rospy.loginfo("Daisy chain port: %s", self.port)
        rospy.loginfo("Daisy node timeout = %s", self.timeout)

        # get device numbers from ros parameter server (see launch~_node.launch)
        self.arm_left_devnum = rospy.get_param("~linear_actuators/arm_left")
        self.arm_right_devnum = rospy.get_param("~linear_actuators/arm_right")
        self.bucket_left_devnum = rospy.get_param("~linear_actuators/bucket_left")
        self.bucket_right_devnum = rospy.get_param("~linear_actuators/bucket_right")
        self.sled_left_devnum = rospy.get_param("~sled/left")
        self.sled_right_devnum = rospy.get_param("~sled/right")

        # initalize Daisy chain serial controllers
        self.arm_left = Daisy(self.arm_left_devnum, port=self.port, flip=True) # first one initialized must set the port
        self.arm_right = Daisy(self.arm_right_devnum, flip=True)
        self.bucket_left = Daisy(self.bucket_left_devnum, flip=True)
        self.bucket_right = Daisy(self.bucket_right_devnum, flip=True)
        self.sled_left = Daisy(self.sled_left_devnum, flip=True)
        self.sled_right = Daisy(self.sled_right_devnum, flip=True)
        self.devices = [self.arm_left, self.arm_right, self.bucket_left, self.bucket_right, self.sled_left, self.sled_right]
        self.dev_names = ["arm_left", "arm_right", "bucket_left", "bucket_right", "sled_left", "sled_right"]

        rospy.sleep(0.1) # wait for params to get set

        # Subscribers
        self.arm_sub = rospy.Subscriber("/arm/vel", Float32, self.arm_vel_callback, queue_size=1)
        self.bucket_sub = rospy.Subscriber("/bucket/vel", Float32, self.bucket_vel_callback, queue_size=1)
        self.sled_sub = rospy.Subscriber("/sled/vel", Float32, self.sled_vel_callback, queue_size=1)
        self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self.limit_pub = rospy.Publisher("/pololu/limit_switch", LimitSwitch, queue_size=1)
        rospy.Timer(rospy.Duration(2), self.publish_diagnostics)
        rospy.Timer(rospy.Duration(0.1), self.publish_limits)

        self._has_showed_message = False  # flag to indicate we have showed the motor shutdown message

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        with self.lock:
          self.arm_left.stop()
          self.arm_right.stop()
          self.bucket_left.stop()
          self.bucket_right.stop()
          self.sled_left.stop()
          self.sled_right.stop()
          #for dev in self.devices:
          #    dev.stop()

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

    def publish_diagnostics(self, event):
        """Generate a diagnostic message of device statuses, voltage levels,
        and limit switch triggers. Not critical code.
        """
        with self.lock:
            diagnostic_arr = DiagnosticArray()  # create object to hold data to publish
            # Generate a ROS message header for time stamping
            header = Header()
            header.stamp = rospy.Time.now()
            diagnostic_arr.header = header

            # For all the devices, set the status based on Pololu serial variables
            # and append these statues to the diagnostic_arr status field
            for i in range(len(self.devices)):
                dev = self.devices[i]
                name = self.dev_names[i]
                status = DiagnosticStatus()
                status.name = name
                status.hardware_id = str(self.devices[i].dev_num)

                errstat = dev.get_error_status()
                sererr = dev.get_serial_errors()
                if errstat == "OK" and sererr == "OK":
                    status.level = 0
                    status.message = errstat + "\n" + sererr
                else:
                    status.level = 1
                    status.message = errstat + "\n" + sererr

                # Get voltage level and set error status if it is too high or low
                voltage_level = dev.get_input_voltage()
                if voltage_level > 30 or voltage_level < 20:
                    status.level = 2
                    status.message = "Voltage out of range: %.2f" % (voltage_level)

                status.values.append(KeyValue("Voltage", "%.2f" % voltage_level))
                status.values.append(KeyValue("Speed", str(dev.get_speed())))

                diagnostic_arr.status.append(status)

            self.diagnostic_pub.publish(diagnostic_arr)

    def publish_limits(self, event):
        """Publish the limit switch status - this is critical code"""
        with self.lock:
          header = Header()
          header.stamp = rospy.Time.now()
          lim_switch1 = False
          lim_switch2 = False;
          for i in range(len(self.devices)):
              dev = self.devices[i]
              name = self.dev_names[i]
              if "sled" in name:
                  lim_switch1, lim_switch2 = dev.get_limit_statuses()
              if lim_switch1 == True or lim_switch2 == True:
                  break

          limit_switch = LimitSwitch()
          limit_switch.front = lim_switch1
          limit_switch.rear = lim_switch2
          self.limit_pub.publish(limit_switch)

    def arm_vel_callback(self, command):
        """Command the arm linear actuators based on the incoming command"""
        with self.lock:
          self.last_set_speed_time = rospy.get_rostime()

          rospy.logdebug("Velocity command to arm linear actuators: %.4f", command.data)

          motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
          motor_command = int(command.data * 3200) # scale to that expected by drivers

          rospy.logdebug("Arm linear actuator serial command = %d", motor_command)

          # drive both linear actuators with the same command
          self.arm_left.drive(motor_command)
          self.arm_right.drive(motor_command)


    def bucket_vel_callback(self, command):
        with self.lock:
          """Command the bucket slinear actuators based on the incoming command"""
          self.last_set_speed_time = rospy.get_rostime()

          rospy.logdebug("Velocity command to bucket linear actuators: %.4f", command.data)

          motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
          motor_command = int(command.data * 3200) # scale to that expected by drivers

          rospy.logdebug("Bucket linear actuator serial command = %d", motor_command)

          # drive both linear actuators with the same command
          self.bucket_left.drive(motor_command)
          self.bucket_right.drive(motor_command)


    def sled_vel_callback(self, command):
        with self.lock:
          """Command the rack and sled based on the incoming command"""
          self.last_set_speed_time = rospy.get_rostime()

          rospy.logdebug("Velocity command to sled motors: %.4f", command.data)

          motor_command = max(-1.0, min(command.data, 1.0)) # put bounds on the incoming command
          motor_command = int(command.data * 3200) # scale to that expected by drivers

          rospy.logdebug("sled motor serial command = %d", motor_command)

          # drive both linear actuators with the same command
          self.sled_left.drive(-motor_command)
          self.sled_right.drive(motor_command)



    def shutdown(self):
        """Handle shutting down the node"""
        rospy.loginfo("Shutting down daisy node")

        if hasattr(self, "sub"):
            # so these don't get called while the node is shutting down
            self.arm_sub.unregister()
            self.bucket_sub.unregister()
            self.sled_sub.unregister()

        self._stop_all_motors()
        # quit()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting daisy node")
