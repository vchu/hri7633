#!/usr/bin/env python
import roslib; roslib.load_manifest('audio_tracking')
import rospy
import numpy as np
import sys
import pyaudio
from audio_tracking.msg import AudioData16

class AudioCaptureRaw():

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.data = []
        # Subscribers
        #rospy.loginfo("Setting up subscribers")
        #rospy.Subscriber("audio", AudioData, self.audio_callback, queue_size = 10)
        
        # Setup publishers
        rospy.loginfo("Setting up publishers")
        self.raw_pub = rospy.Publisher('audio_raw', AudioData16)

        # Setup the recording
        self.chunk = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
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

        #data = self.stream.read(self.chunk)
        #data_int = np.fromstring(data, dtype=np.int16)
        data_int = np.fromstring(in_data, dtype=np.int16)
        msg_audio = AudioData16()
        msg_audio.data = data_int
        self.raw_pub.publish(msg_audio)
       
        return None, pyaudio.paContinue
 
    def sample_audio(self):
        # Setup rate
        r = rospy.Rate(1) # 30 hz
        rospy.loginfo("Start streaming")

        while not rospy.is_shutdown():
            data = self.stream.read(self.chunk)
            data_int = np.fromstring(data, dtype=np.int16)
            msg_audio = AudioData16()
            msg_audio.data = data_int
            self.raw_pub.publish(msg_audio)

    def cleanup(self):

        rospy.loginfo("Exiting")


def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('audio_learning_capture', anonymous=True)
    rospy.loginfo("Audio Capture Node Started")
    audio_raw = AudioCaptureRaw()
    #audio_raw.sample_audio()
    rospy.spin()

if __name__ == '__main__':
    main()




