#!/usr/bin/env python
import roslib; roslib.load_manifest('bluetooth_capture')
import rospy
import os
import time
import subprocess
import signal
from std_msgs.msg import Bool
from data_logger_bag.msg import LogControl

class BluetoothSniffer():
    
    ''' 
    Initialize the node 
    '''
    def __init__(self):

        self.userName = None

        # Initialize node
        rospy.init_node("bluetooth_sniffer", anonymous=True)
        rospy.loginfo("Initializing bluetooth sniffer node")

        # Create subscriber that listens for when people have
        # been asked to play music on their phones
        rospy.Subscriber("bluetooth_sniffer_flag", Bool, self.bluetooth_callback)

        # Optional subscriber that figures out user name?
        # rospy.Subscriber("C6_Task_Description", LogControl, self.log_callback)

        # Setup publisher to show when done writing the file?
        self.pub = rospy.Publisher("bluetooth_done_flag", Bool)

    def bluetooth_callback(self, msg):

        # Create the ubertooth command
        ubertooth_cmd = "sudo ./ubertooth-scan"
        if self.userName != None:
            ubertooth_cmd = ubertooth_cmd + ' -f ' + self.userName + '.txt'

        # Calling the ubertooth command 
        rospy.loginfo("Command to run: %s" % ubertooth_cmd)

        # Start the command through the system
        self.uber_proc = subprocess.Popen([ubertooth_cmd], shell=True)
        rospy.loginfo("Running bluetooth scan")

        # Should wait until it finishes
        self.uber_proc.wait()
        
        # Let the program know that the bluetooth scan has finished
        self.pub.publish(True)

    def log_callback(self, msg):
        '''
        Currently hardcoded in the system that skillName will contain the username
        '''

        if msg.skillName is not "": 
            self.user = msg.skillName
            rospy.loginfo("User is: %s" % self.user)


if __name__ == '__main__':

    bt = BluetoothSniffer()
    rospy.spin() 
 
