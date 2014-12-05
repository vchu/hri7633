#!/usr/bin/env python
import roslib; roslib.load_manifest('bluetooth_capture')
import rospy
import os
import time
import subprocess
import signal
from std_msgs.msg import Bool

class BluetoothSniffer():
    
    ''' 
    Initialize the node 
    '''
    def __init__(self):

        # Initialize node
        rospy.init_node("bluetooth_sniffer", anonymous=True)
        rospy.loginfo("Initializing bluetooth sniffer node")

        # Create subscriber that listens for when people have
        # been asked to play music on their phones
        rospy.Subscriber("bluetooth_sniffer_flag", Bool, self.bluetooth_callback)

        # TODO: Optional subscriber that figures out user name?

        # Setup publisher to show when done writing the file?
        self.pub = rospy.Publisher("bluetooth_done_flag", Bool)

    def bluetooth_callback(self):

        # Calling the ubertooth command 
        ubertooth_cmd = "./ubertooth-scan"
        rospy.loginfo("Command to run: %s" % ubertooth_cmd)

        # Start the command through the system
        self.uber_proc = subprocess.Popen([ubertooth_cmd], shell=True)
        rospy.loginfo("Running bluetooth scan")



if __name__ == '__main__':

    bt = BluetoothSniffer()
 
 
