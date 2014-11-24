#!/usr/bin/env python
import roslib; roslib.load_manifest('audio_capture')
import rospy
from audio_common_msgs.msg import AudioData


class AudioCaptureProcessing():

    def __init__(self):

        # Subscribers
        rospy.loginfo("Setting up subscribers")
        rospy.Subscriber("contingent_data_logger_flag", Bool, self.callback)
        rospy.Subscriber("audio", AudioData, self.audio_callback, queue_size = 1000)
        
        # Setup publishers
        rospy.loginfo("Setting up publishers")
        self.raw_amp = rospy.Publisher('audio_amp', Float64)


    def audio_callback(self, msg):

        # Do something with audio?
        print msg.data 



def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('audio_learning_capture', anonymous=True)
    rospy.loginfo("Audio Processing Node Started")
    AudioCaptureProcessing()
    rospy.spin()

if __name__ == '__main__':
    main()




