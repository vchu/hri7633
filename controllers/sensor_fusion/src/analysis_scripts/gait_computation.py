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

def process_data_usr(data):

    import pdb; pdb.set_trace()

    all_data = data['data_set_single']
    gait_data = dict()
    test_data = dict()

    for datatype in all_data:

        type_data = all_data[datatype]

        # Only want to "train" on the first two runs
        if datatype == 'lab_member':
            for user in type_data:

                user_data = type_data[user]
                if user not in test_data:
                    test_data[user] = []

                for run_name in user_data:

                    run_data = user_data[run_name]['gait_tracking']
                    best_user = ''
                    num_points = 0
                  
                    # Find the "best" - which is currently the person with the most values 
                    for userID in run_data:
                        head_translation = run_data[userID]['head']['translation'] 
                        if len(head_translation) > num_points:
                            best_user = userID
                            num_points = len(head_translation) 

                    test_data[user].append(run_data[userID])
            
        else:

            for user in type_data:

                user_data = type_data[user]
                if user not in gait_data:
                    gait_data[user] = []

                for run_name in user_data:

                    run_data = user_data[run_name]['gait_tracking']
                    best_user = ''
                    num_points = 0
                  
                    # Find the "best" - which is currently the person with the most values 
                    for userID in run_data:
                        head_translation = run_data[userID]['head']['translation'] 
                        if len(head_translation) > num_points:
                            best_user = userID
                            num_points = len(head_translation) 

                    gait_data[user].append(run_data[userID])

    # store the data...
    cPickle.dump(gait_data, open("gait_data.pkl", "wb" ), protocol=cPickle.HIGHEST_PROTOCOL)
    cPickle.dump(test_data, open("test_gait_data.pkl", "wb" ), protocol=cPickle.HIGHEST_PROTOCOL)

    import pdb; pdb.set_trace()

def process_data(data):

    import pdb; pdb.set_trace()

    all_data = data['data_set_single']
    gait_data = dict()

    for datatype in all_data:

        type_data = all_data[datatype]
        # Only want to "train" on the first two runs
        #if datatype == 'lab_member':
        #    break

        if datatype not in gait_data:
            gait_data[datatype] = []

        for user in type_data:

            user_data = type_data[user]
            for run_name in user_data:

                run_data = user_data[run_name]['gait_tracking']
                best_user = ''
                num_points = 0
              
                # Find the "best" - which is currently the person with the most values 
                for userID in run_data:
                    head_translation = run_data[userID]['head']['translation'] 
                    if len(head_translation) > num_points:
                        best_user = userID
                        num_points = len(head_translation) 

                gait_data[datatype].append(run_data[userID])

    # store the data...
    cPickle.dump(gait_data, open("gait_data_datatype.pkl", "wb" ), protocol=cPickle.HIGHEST_PROTOCOL)

    import pdb; pdb.set_trace()

def main():

    if len(sys.argv) == 2:
        input_file = sys.argv[1]
        all_data = load_data(input_file, '', False)

        process_data_usr(all_data)

    else:
        print 'error'

if __name__== "__main__":
    main()

