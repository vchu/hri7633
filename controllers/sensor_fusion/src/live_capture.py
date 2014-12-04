#!/usr/bin/env python
import roslib; roslib.load_manifest('sensor_fusion')
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt 
import threading
from collections import defaultdict
from audio_tracking.msg import AudioData16
from gait_capture.msg import PersonFrame

class SensorFusion():

    def __init__(self):

        rospy.loginfo("Initializing sensor fusion node")
        # testing shutdown
        #rospy.on_shutdown(self.cleanup)

        # Locks to access data
        self.audio_lock = threading.Lock()
        self.gait_lock = threading.Lock()

        # Setup Subscribers
        rospy.loginfo("Setting up subscribers")
        rospy.Subscriber("audio_raw", AudioData16, self.audio_callback, queue_size = 10) 
        rospy.Subscriber("gait_tracking", PersonFrame, self.gait_callback, queue_size = 10) 

        # Initialize some audio values
        self.audio_data = [] # Can store last X number of msgs
        self.gait_data = defaultdict(dict)

    def audio_callback(self, msg):

        # We only store the last ten seconds (44.1 hz * 10 = ~441 msgs)
        if len(self.audio_data) > 450:
            self.audio_data.pop(0) # throw it out

        # Get lock...
        # This implementation might cause some msgs to get lost....
        if not self.audio_lock.acquire():
            rospy.loginfo("Failed to acquire lock - audio")
        else:
            try:
                self.audio_data.append(msg)
            finally:
                self.audio_lock.release()


    def gait_callback(self, msg):

        # Pull out ID
        ID = msg.person_id

        # Pull out body_parts
        person_frame = msg.body_parts

        # Get lock...
        # This implementation might cause some msgs to get lost....
        if not self.gait_lock.acquire():
            rospy.loginfo("Failed to acquire lock - gait")
        else:
            try:

                if 'body_parts' not in self.gait_data[ID]:
                    self.gait_data[ID]['latest_time'] = []

                # Process the gait such that it is stored in a dictionary of transforms?
                self.gait_data[ID]['latest_time'].append(msg.latest_time)

                for body_part in person_frame:

                    body_part_name = '_'.join(body_part.child_frame_id.split('_')[0:-1])[1::]
                    # Check if the body part exists first
                    if body_part_name not in self.gait_data[ID]:
                        self.gait_data[ID][body_part_name] = []

                    # Put into dictionary 
                    self.gait_data[ID][body_part_name].append(body_part.transform)

            finally:
                self.gait_lock.release()

    def cleanup(self):

        print 'numpy!'
        import pdb; pdb.set_trace()



def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('sensor_fusion', anonymous=True)
    rospy.loginfo("Sensor Fusion Node Started")
    SensorFusion()
    rospy.spin()

if __name__ == '__main__':
    main()

