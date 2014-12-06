#!/usr/bin/env python
import roslib; roslib.load_manifest('audio_tracking')
import rospy
import numpy as np
import sys
import pyaudio
from audio_common_msgs.msg import AudioData

class AudioCaptureRaw():

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.data = []
        
        # Setup publishers
        rospy.loginfo("Setting up publishers")
        self.raw_pub = rospy.Publisher('audio_raw', AudioData)

        # Setup the recording
        self.chunk = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 2
        RATE = 44100

        self.p = pyaudio.PyAudio()

        self.stream = self.p.open(format = FORMAT,
                        channels = CHANNELS,
                        rate = RATE,
                        input = True,
                        frames_per_buffer = self.chunk,
                        stream_callback=self.sample_audio_test)

        rospy.loginfo("Done Initialization")

    def sample_audio_test(self, in_data, frame_count, time_info, status):

        #data_int = np.fromstring(in_data, dtype=np.int16)
        #msg_audio = AudioData16()
        #msg_audio.data = data_int

        msg_audio = AudioData()
        msg_audio.data = in_data
        self.raw_pub.publish(msg_audio)
       
        return None, pyaudio.paContinue

    def cleanup(self):

        rospy.loginfo("Exiting")


def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('audio_learning_capture', anonymous=True)
    rospy.loginfo("Audio Capture Node Started")
    audio_raw = AudioCaptureRaw()
    rospy.spin()

if __name__ == '__main__':
    main()




