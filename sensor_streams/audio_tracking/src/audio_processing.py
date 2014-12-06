#!/usr/bin/env python
import roslib; roslib.load_manifest('audio_tracking')
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt
import pyaudio
import wave
from std_msgs.msg import Float64
from audio_common_msgs.msg import AudioData

class AudioCaptureProcessing():

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.data = []
        # Subscribers
        rospy.loginfo("Setting up subscribers")
        rospy.Subscriber("audio_raw", AudioData, self.audio_callback, queue_size = 10)
        
        # Setup publishers
        #rospy.loginfo("Setting up publishers")
        #self.raw_amp = rospy.Publisher('audio_raw', AudioData16)

        # Setup some calibration values



    def audio_callback(self, msg):

        # Do something with audio?
        raw_audio = msg.data

        '''
        # Split data into left and right channels
        right_audio, left_audio = raw_audio[0::2],raw_audio[1::2]

        # Grab largest values
        left_max = max(np.absolute(left_audio)) - 250
        right_max = max(np.absolute(right_audio))

        #print 'right: %d' % right_max
        #print 'left: %d' % left_max

        threshold = 300

        if abs(left_max - right_max) > threshold:
            if left_max > right_max:
                print "left louder"
            else:
                print "right_louder"
        '''
        '''
        audio_data = [struct.unpack('B',i[0])[0] for i in raw_audio]
        audio_stream = np.array(audio_data)
        
        # Write data to mp3 file
        data_bytes = bytearray(audio_stream)
        temp_file_name = 'temp_audio.mp3'
        mp3_file = open(temp_file_name,'w')
        mp3_file.write(''.join(map(chr,data_bytes)))
        mp3_file.close()

        import pdb; pdb.set_trace()
        # Read the file back in 
        datastream = AudioSegment.from_mp3(temp_file_name)

        # Grab the raw data and convert it into an np array 
        raw_audio_bytes = datastream._data
        converted_audio = np.fromstring(raw_audio_bytes, dtype=np.int16)

 

        print len(left_audio)
        print len(right_audio)

        '''
        self.data.append(msg.data)

    def cleanup(self):

        rospy.loginfo("numpy!")
        cur_data = self.data[0:100]

        chunk = 1024
        FORMAT = pyaudio.paInt16
        RATE = 44100
        RECORD_SECONDS = 5 
        WAVE_OUTPUT_FILENAME = "output.wav"

        p = pyaudio.PyAudio()

        wf = wave.open('test.wav', 'wb')
        wf.setnchannels(2)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(cur_data))
        wf.close()


        import pdb; pdb.set_trace()


def main():

    # Anonymous = true means that the node has a unique identifier
    rospy.init_node('audio_learning_capture', anonymous=True)
    rospy.loginfo("Audio Processing Node Started")
    AudioCaptureProcessing()
    rospy.spin()

if __name__ == '__main__':
    main()




