#!/usr/bin/python
from __future__ import division
from pololu_driver import clip, Daisy # serial controller for controller in Daisy chain

import rospy
import threading
from std_msgs.msg import Float32, Header, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# TODO: if necessary, add more try and excepts for error catching

class FreshVal(object):
    """Holds a value and returns the timeout val if the data gets old"""
    def __init__(self, stale_val=0, timeout=None, time=rospy.Time.now(), name=""):
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
    def val(self):
        # if stale, return timeout_val
        if (rospy.Time.now() - self.last_update_time).to_sec() > self.timeout:
            if not self._has_shown_message: 
                rospy.logdebug("[POLOLU] %s value timed out %d", self.name, self._val)
                self._has_shown_message = True
            return self.stale_val
        else:
            self._has_shown_message = False
            return self._val

class Node(object):
    def __init__(self):
        """Init ros node"""
        rospy.init_node("pololu_node", log_level=rospy.DEBUG) 
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[POLOLU]: Connecting to pololu daisy chain")
        self.last_set_speed_time = rospy.get_rostime()
        self.port = rospy.get_param("~port")
        self.timeout = rospy.get_param("~timeout")  # time between hearing commands before we shut off the motors
        self.last_insert_msg = FreshVal(stale_val=0, timeout=self.timeout, name="insert")
        self.last_extend_msg = FreshVal(stale_val=0, timeout=self.timeout, name="extend")
        self.last_spin_msg =   FreshVal(stale_val=0, timeout=self.timeout, name="spin")

        rospy.loginfo("[POLOLU] chain port: %s", self.port)
        rospy.loginfo("[POLOLU] timeout = %s", self.timeout)

        # get device numbers (see daisy_node.launch) and open a serial connection
        # (la = linear_actuator)
        self.insert_la =       Daisy(rospy.get_param("~dev_num/insert_la"), port=self.port) # 1st one sets port
        self.extend_la_left =  Daisy(rospy.get_param("~dev_num/extend_la_left"))
        self.extend_la_right = Daisy(rospy.get_param("~dev_num/extend_la_right"))
        self.dump_spin_left =  Daisy(rospy.get_param("~dev_num/dump_spin_left"))
        self.dump_spin_right = Daisy(rospy.get_param("~dev_num/dump_spin_right"))

        self.devices = [self.insert_linear, self.extend_linear_left, self.extend_linear_left, self.dump_spin_left, self.dump_spin_right]

        # TODO (p1): add some checks to make sure that devices boot up 

        rospy.sleep(0.1) # wait for params to get set

        # Subscribers
        self.insert_sub = rospy.Subscriber("insert_linear", Float32, self.insert_callback, queue_size=1)
        self.extend_sub = rospy.Subscriber("extend_linear", Float32, self.extend_callback, queue_size=1)
        self.dump_sub =   rospy.Subscriber("dump_spin", Float32, self.dump_callback, queue_size=1)
        self.digger_extended_pub = rospy.Publisher("/digger_extended", Bool, queue_size=1)
        #self.diagnostic_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        #rospy.Timer(rospy.Duration(0.1), self.publish_limits)
        self._have_shown_message = False  # flag to indicate we have showed the motor shutdown message

    def _stop_all_motors(self):
        """Send stop command to all daisy chained motors"""
        for dev in self.devices:
            dev.stop()

    def _send_insert_cmd(self):
        # TODO (p1): add a curself.insert_cmd.valrent check here to shut down motor if it gets too high 
        last_cmd = self.insert_cmd.val
        self.insert_la.drive(last_cmd)

    def _send_extend_cmd(self):
        last_cmd = self.last_extend_mgs.val # make sure they get the same command
        self.extend_la_left.drive(last_cmd)
        self.extend_la_right.drive(last_cmd)

    def _send_spin_cmd(self):
        """Send spin command to the dumping motors"""
        last_cmd = self.last_spin_msg.val # make sure they get the same command
        self.spin_left.drive(last_cmd)
        self.spin_right.drive(-last_cmd)

    def _read_la_states(self):
        insert_la_state = self.insert_la.get_an1()
        insert_la_current = self.insert_la.get_an2()
        extend_la_left_state = self.extend_la_left.get_an1()
        extend_la_right_state = self.extend_la_right.get_an1()

        # TODO (p1): add logic 
        rospy.loginfo("insert an1 = %d", insert_la_state)
        rospy.loginfo("insert an2 = %d", insert_la_current)
        rospy.loginfo("extend left an1 = %d", extend_la_left_state)
        rospy.loginfo("extend right an2 = %d", extend_la_right_state)
    
    def run(self):
        """Run the main ros loop. All Serial communication should happen in this thread"""
        rospy.loginfo("[POLOLU]: Starting node loop")
        rate = rospy.Rate(30) 

        while not rospy.is_shutdown():
            self._send_insert_cmd()
            self._send_extend_cmd()
            self._send_spin_cmd()

            self._read_la_states()

            rate.sleep()

    def _scale_and_clip(cmd):
        """[-1,1] --> [-3200, 3200] for (ROS --> Pololu SMC)"""
        return clip(int(cmd.data * 3200), -3200, 3200)

    # ROS command callbacks
    def insert_callback(self, msg):
        """Update insert command value"""
        rospy.logdebug("[POLOLU]: Velocity cmd to insert linear actuator: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_insert_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Insert linear actuator serial cmd = %d", cmd)

    def extend_callback(self, msg):
        rospy.logdebug("[POLOLU]: Velocity cmd to extend linear actuators: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_extend_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Extend linear actuator serial cmd = %d", cmd)

    def dump_callback(self, msg):
        rospy.logdebug("[POLOLU]: Velocity cmd to dump spin motors: %.4f", msg.data)
        cmd = self._scale_and_clip(msg)
        self.last_dump_msg.update(cmd)
        rospy.logdebug("[POLOLU]: Dump linear actuator serial cmd = %d", cmd)

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
    rospy.loginfo("[POLOLU]: Exiting node")
