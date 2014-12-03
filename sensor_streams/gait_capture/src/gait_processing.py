#!/usr/bin/env python
import roslib; roslib.load_manifest('gait_capture')
import rospy
import struct
import numpy as np
import copy
import matplotlib.pyplot as plt
from collections import defaultdict
from tf.msg import tfMessage
from gait_capture.msg import PersonFrame

class GaitCaptureProcessing():

    def __init__(self):

        # If we want exit behavior
        # rospy.on_shutdown(self.cleanup)

        # Data storage?
        self.data = defaultdict(dict)
        self.body_tracker = defaultdict(dict)
 
        # Setup publisher
        rospy.loginfo("Setting up publishers")
        self.person_pub = rospy.Publisher("gait_tracking", PersonFrame)

        # Subscribers
        rospy.loginfo("Setting up subscribers")
        rospy.Subscriber("tf", tfMessage, self.gait_callback, queue_size = 10)


    def gait_callback(self, msg):

        # Throw away data not from the tracker
        if msg.transforms[0].header.frame_id != '/openni_depth_frame':
            return

        tfs = msg.transforms[0]
        ID = int(tfs.child_frame_id.split('_')[-1])
        body_part = '_'.join(tfs.child_frame_id.split('_')[0:-1])

        # Create new dictionary when head appears
        if 'head' in body_part:
            self.body_tracker[ID] = dict()
            self.body_tracker[ID]['latest_time'] = rospy.Time(10)

        # Skip if we haven't seen a head yet...
        if 'latest_time' not in self.body_tracker[ID]:
            return

        latest_time = self.body_tracker[ID]['latest_time']
        # Check to guarantee body parts are from similar time
        if tfs.header.stamp < latest_time - rospy.Duration(0.001):
            return

        # Store off body part and time stamp if newer
        self.body_tracker[ID][body_part] = tfs 
        if latest_time > tfs.header.stamp:
            self.body_tracker[ID]['latest_time'] = latest_time
        else:
            self.body_tracker[ID]['latest_time'] = tfs.header.stamp

        # Trigger adding of frame if right_foot is seen
        # We use time above to regulate that the body pose hasn't significantly changed..
        #if 'right_foot' in body_part and '/head' in self.body_tracker[ID].keys():
        if len(self.body_tracker[ID]) == 16:
            self.process_frame(ID, self.body_tracker[ID])

    def process_frame(self, person_id, frame):

        # Create message to send
        person_msg = PersonFrame()
        person_msg.body_parts = []
        person_msg.person_id = person_id

        # Store the frame away currently
        if person_id not in self.data:
            self.data[person_id] = defaultdict(dict)

        for body_part in frame: 

            # Skip time key
            if 'time' in body_part:
                person_msg.latest_time = frame[body_part]
                continue

            if body_part not in self.data[person_id]:
                self.data[person_id][body_part] = []

            #self.data[person_id][body_part].append(frame[body_part].transform)
            person_msg.body_parts.append(frame[body_part]) 

        self.person_pub.publish(person_msg)    

    def cleanup(self):

        rospy.loginfo("numpy!")
        temp = self.data
        for body_part in temp[1]:
            print len(temp[1][body_part])


def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('gait_learning_capture', anonymous=True)
    rospy.loginfo("Gait Processing Node Started")
    GaitCaptureProcessing()
    rospy.spin()

if __name__ == '__main__':
    main()




