#!/usr/bin/env python
# Script to start loading data into pytables and convert into meaningful features
import roslib; roslib.load_manifest("data_logger_bag")
import rospy
import sys
import tables
import numpy as np
import cPickle
import csv
from collections import defaultdict
import matplotlib.pyplot as plt
from load_h5_dataset import load_data

def process_data(data):

    import pdb; pdb.set_trace()

    all_data = data['data_set_single']

    table_set_1 = ['user01']
    table_set_2 = ['user02']

    height_data = dict()

    for datatype in all_data:

        type_data = all_data[datatype]
        #if datatype == 'lab_member':
        #    break
        for user in type_data:

            user_data = type_data[user]
            for run_name in user_data:

                if user not in height_data:
                    height_data[user] = []

                run_data = user_data[run_name]['gait_tracking']
                best_user = ''
                num_points = 0
              
                # Find the "best" - which is currently the person with the most values 
                for userID in run_data:
                    head_translation = run_data[userID]['head']['translation'] 
                    if len(head_translation) > num_points:
                        best_user = userID
                        num_points = len(head_translation) 
                height_data[user].append(run_data[userID]['head']['translation'].tolist())

    # store the data...
    cPickle.dump(height_data, open("gait_height.pkl", "wb" ), protocol=cPickle.HIGHEST_PROTOCOL)

    import pdb; pdb.set_trace()

def main():

    if len(sys.argv) == 2:
        input_file = sys.argv[1]
        all_data = load_data(input_file, '', False)

        process_data(all_data)

    else:
        print 'error'

if __name__== "__main__":
    main()

