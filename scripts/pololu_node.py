#!/usr/bin/python
from __future__ import division
from pololu_driver import clip, Pololu # serial controller for Pololu

import rospy
import threading
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TODO: if necessary, add more try and excepts for error catching

class FreshVal(object):
    """Holds a value and returns the timeout val if the data gets old"""
    def __init__(self, stale_val=0, timeout=None, time=rospy.Time(), name=""):
        if timeout is None:
            raise ValueError("timeout must be set")
        self.timeout = timeout
        self.stale_val = stale_val
        self._val = self.stale_val # initialize the value to stale
        self.last_update_time = time
        self.name = name

        self._has_shown_message = False

    def update(self, val):
        self._val = val 
        self.last_update_time = rospy.Time.now()

    @property
    def is_fresh(self):
        fresh = True
        fresh = fresh and self._val is not None
        fresh = fresh and (rospy.Time.now() - self.last_update_time).to_sec() < self.timeout
        return fresh

    @property
    def val(self):
        # if stale, return timeout_val
        if (rospy.Time.now() - self.last_update_time).to_sec() > self.timeout:
            if not self._has_shown_message: 
                rospy.logdebug("[POLOLU]: %s value timed out %d", self.name, self._val)
                self._has_shown_message = True
            return self.stale_val
        else:
            self._has_shown_message = False
            return self._val

class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node", log_level=rospy.INFO) 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[POLOLU]: Connecting to pololu daisy chain")
        self.last_set_speed_time = rospy.get_rostime()
        self.timeout = rospy.get_param("~timeout")  # time between hearing commands before we shut off the motors
        self.extend_state = rospy.get_param("~extend_state")
        self.max_accel = int(rospy.get_param("~max_accel"))

        # Commands and actuator state variables
        self.last_extend_msg = FreshVal(stale_val=0, timeout=self.timeout, name="extend")
        self.last_insert_msg = FreshVal(stale_val=0, timeout=self.timeout, name="insert")
        self.last_spin_msg =   FreshVal(stale_val=0, timeout=self.timeout, name="spin")
        self.ela_state =       FreshVal(stale_val=0, timeout=self.timeout, name="extend_state")
        self.ila_left_state =  FreshVal(stale_val=0, timeout=self.timeout, name="insert_left_state")
        self.ila_right_state = FreshVal(stale_val=0, timeout=self.timeout, name="insert_right_state")

        rospy.loginfo("[POLOLU]: timeout = %s", self.timeout)

        # get device numbers (see daisy_node.launch) and open a serial connection
        # (la = linear_actuator)
        # (ela = extend_linear_actuator)
        # (ila = insert_linear_actuator)
        self.ela =       Pololu(rospy.get_param("~dev_num/extend_la"), flip=True)
        # NOTE: THESE MUST HAVE THE SAME FLIP
        self.ila_left =  Pololu(rospy.get_param("~dev_num/insert_la_left"), flip=True)
        self.ila_right = Pololu(rospy.get_param("~dev_num/insert_la_right"), flip=True)
        self.dumper_spin_left =  Pololu(rospy.get_param("~dev_num/dumper_spin_left"))
        self.dumper_spin_right = Pololu(rospy.get_param("~dev_num/dumper_spin_right"))

        self.devices = [self.ela, self.ila_left, self.ila_right, self.dumper_spin_left, self.dumper_spin_right]
        self._check_devices()
        self._set_accel_limits()

        rospy.sleep(0.1) # wait for params to get set

        # Subscribers
        self.extend_sub = rospy.Subscriber("/extend_la/cmd", Float32, self.extend_callback, queue_size=1)
        self.insert_sub = rospy.Subscriber("/insert_la/cmd", Float32, self.insert_callback, queue_size=1)
        self.dumper_sub = rospy.Subscriber("/dumper_spin/cmd", Float32, self.dumper_callback, queue_size=1)
        self.extend_state_pub = rospy.Publisher("/extend_la/state", Float32, queue_size=1)
        self.insert_state_pub = rospy.Publisher("/insert_la/state", Float32, queue_size=1)
        self.digger_extended_pub = rospy.Publisher("/digger_extended", Bool, queue_size=1)
        self._have_shown_message = False  # flag to indicate we have showed the motor shutdown message

    def _set_accel_limits(self):
        for dev in self.devices:
            dev.set_motor_limits(self.max_accel, self.max_accel)

        rospy.loginfo("[POLOLU]: Set acceleration limit to ({}/3200)".format(self.max_accel))

    def _check_devices(self): 
        """just check that you can read from a device and that you get the
        expected value. this will crash cause the node to crash if something
        is up"""
        for dev in self.devices:
            baud = dev.get_baud()
            if baud != 625: # 625 means 115200 (in manual, you divide 72e6/baud)
                raise Exception("Read BAUD=={} from device {}".format(baud, dev.dev_num))
        rospy.loginfo("[POLOLU]: Devices connected")

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        for dev in self.devices:
            dev.stop()

    def _send_extend_cmd(self):
        last_cmd = self.last_extend_msg.val
        # TODO (p1): add a check here to monitor that the la is not jamming.
        self.ela.drive(last_cmd)

    def _send_insert_cmd(self):
        last_cmd = self.last_insert_msg.val # make sure they get the same command
        self.ila_left.drive(last_cmd)
        self.ila_right.drive(last_cmd)

    def _send_spin_cmd(self):
        """Send spin command to the dumping motors"""
        # TODO: replace this nerfed value with acceleration limits
        last_cmd = int(1.0*self.last_spin_msg.val) # make sure they get the same command
        self.dumper_spin_left.drive(last_cmd)
        self.dumper_spin_right.drive(-last_cmd)

    def _read_la_states(self):
        # read from serial and update variables
        self.ela_state.update(self.ela.get_an1())
        self.ila_left_state.update(self.ila_left.get_an1())
        self.ila_right_state.update(self.ila_right.get_an1())
        # debug msg
        rospy.logdebug("[POLOLU]: extend linear actuator state = {}".format(self.ela_state.val))
        rospy.logdebug("[POLOLU]: insert linear actuator left state = {}".format(self.ila_left_state.val))
        rospy.logdebug("[POLOLU]: insert linear actuator right state = {}".format(self.ila_right_state.val))

    def _publish_la_states(self):
        # TODO (p2): convert these to meters once they are attached to the robot 
        # publish extend linear actuator state
        if self.ela_state.is_fresh:
            self.extend_state_pub.publish(self.ela_state.val)
        # publish insert linear actuator state
        if self.ila_right_state.is_fresh and self.ila_left_state.is_fresh:
            val = 0.5*(self.ila_left_state.val + self.ila_right_state.val)
            self.insert_state_pub.publish(val)

            # TODO (p1) : uncomment and test this when stuff is wired
            ## if we are extended (this is received by roboclaw_node to make sure we never dig when we are not extended)
            #if val > self.extended_state:
            #    self.digger_extended_pub.publish(True)
            #else:
            #    self.digger_extended_pub.publish(False)

    def run(self):
        """Run the main ros loop. All Serial communication should happen in this thread"""
        rospy.loginfo("[POLOLU]: Starting node loop")
        rate = rospy.Rate(30) 

        while not rospy.is_shutdown():
            # Write
            self._send_extend_cmd()
            self._send_insert_cmd()
            self._send_spin_cmd()
            # Read
            #self._read_la_states()
            # ROS publish
            #self._publish_la_states()

            rate.sleep()

    def _scale_and_clip(self, cmd):
        """[-1,1] --> [-3200, 3200] for (ROS --> Pololu SMC)"""
        return clip(int(cmd.data * 3200), -3200, 3200)

    # ROS command callbacks
    def extend_callback(self, msg):
        """Update extend command value"""
        rospy.logdebug("[POLOLU]: Velocity cmd to extend linear actuator: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_extend_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Insert linear actuator serial cmd = %d", cmd)

    def insert_callback(self, msg):
        rospy.logdebug("[POLOLU]: Velocity cmd to insert linear actuators: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_insert_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Extend linear actuator serial cmd = %d", cmd)

    def dumper_callback(self, msg):
        rospy.logdebug("[POLOLU]: Velocity cmd to dumper spin motors: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_spin_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Dump spin serial cmd = %d", cmd)

    def shutdown(self):
        """Handle shutting down the node"""
        rospy.loginfo("Shutting down daisy node")
        # so these don't get called while the node is shutting down
        # (need to add checks in case it immediately shuts down. (i think))
        if hasattr(self, "extend_sub"):
            self.extend_sub.unregister()
        if hasattr(self, "insert_sub"):
            self.insert_sub.unregister()
        if hasattr(self, "dumper_sub"):
            self.dumper_sub.unregister()
        self._stop_all_motors()

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("[POLOLU]: Exiting node")
