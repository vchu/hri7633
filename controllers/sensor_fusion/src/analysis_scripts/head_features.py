#!/usr/bin/env python
# Script to start loading data into pytables and convert into meaningful features
import roslib; roslib.load_manifest("data_logger_bag")
import rospy
import sys
import tables
from itertools import cycle
import numpy as np
import cPickle
import csv
import math
import random
from scipy import signal
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from load_h5_dataset import load_data
from matplotlib.lines import Line2D

def load_pkl(filename):

    data_store = cPickle.load(open(filename, "rb" ))

    fig_num = 0
    #colors = ['r','g','b','y','c','m','k']
    colors = cm.rainbow(np.linspace(0, 1, 12))
    lines = ["-","--","-.",":"]
    markers = [',','o','v', 's','D', 'p','h','<','>','*']
    colorcycler = cycle(colors)
    linecycler = cycle(lines)
    markercycler = cycle(markers)
    fig = plt.figure(fig_num, figsize=(15,10))
    plt.ylim([-0.5,0.5])
    #plt.ylim([0,0.4])
    all_users = ['user01','user02','user03','user04','user05','user06','user07',
    'user08','user09','user13','user14','user15']
    plot_lines = []

    '''
    markers = []
    for m in Line2D.markers:
        try:
            if len(m) == 1 and m != ' ':
                markers.append(m)
        except TypeError:
            pass
    styles = markers + [
        r'$\lambda$',
        r'$\bowtie$',
        r'$\circlearrowleft$',
        r'$\clubsuit$',
        r'$\checkmark$']
    markercycler = cycle(styles)
    '''
    random.shuffle(markers)

    for user in all_users:
        user_data = data_store[user]
        #fig = plt.figure(fig_num)
        color_type = next(colorcycler)
        line_type = next(linecycler)
        marker_type = next(markercycler)
        z_data_all = []
        x_data_all = []
        for data in user_data:
            #line_type = next(linecycler)     
            #color_type = next(colorcycler)     
            # Compute moving average
            #data_test = moving_average(data, n= 20)
            #data_test = remove_spikes(data)
            #data_test = use_last_good(data)
            #plt.plot(data, line_type, color_type)
            #plt.plot(data_test, color_type)
            #plt.plot(data, color_type+line_type)
            #plt.plot([np.mean(data[200:1000])]*100, color_type+line_type)

            '''
            # median filter on the raw height (head)
            # At 30hz so convert samples to time...
            z_data = np.array(data)[:,2].tolist()
            t = np.linspace(0,len(z_data)/30,len(z_data))
            data_test = signal.medfilt(z_data, kernel_size=701)
            line_created, = plt.plot(t, data_test, color_type+line_type)
            #line_created, = plt.plot(t, data_test, color=color_type, linestyle='', marker=marker_type, set_markerevery(10))
            plot_lines.append(line_created)
            '''

            # Scatter plot of the depth vs. height of the head
            z_data = np.array(data)[:,2].tolist()[0::10]
            x_data = np.array(data)[:,0].tolist()[0::10]
            line_created = plt.scatter(x_data, z_data, c=color_type, marker=marker_type, s=50)
            plot_lines.append(line_created)

            # Create simple regression
            #z_data = np.array(data)[:,2].tolist()
            #x_data = np.array(data)[:,0].tolist()[0::10]
            #z_data_all.append(z_data)
            #x_data_all.append(x_data)
            #z_data = np.array(data)[:,2].tolist()[0::10]
            #x_data = np.array(data)[:,0].tolist()[0::10]
            #data_test = signal.medfilt(z_data, kernel_size=701)
            #plt.plot(x_data, color_type+line_type)
            #plt.scatter(x_data, z_data, c=color_type, marker=marker_type)
            #plt.plot([np.mean(data_test)]*100, color_type+line_type)
            #plt.plot(data_test, color_type+line_type)

        #coef = np.polyfit(x_data, z_data, 1)
        #polynomial = np.poly1d(coef)
        #ys = polynomial(x_data)

        #plt.plot(x_data,ys, color_type+line_type)
        #fig.savefig('height_'+user+'.png', bbox_inches='tight')
        #fig_num+=1

    '''
    plt.xlabel('Time (Sec)', fontsize=18, labelpad=10)
    plt.ylabel('Head Height (Meters)', fontsize=18, labelpad=10)
    fig.legend(plot_lines[::2], all_users, loc='upper right')
    fig.savefig('all_height_median.png', bbox_inches='tight')
    ''' 

    plt.xlabel('Depth (Meters)', fontsize=18, labelpad=10)
    plt.ylabel('Height (Meters)', fontsize=18, labelpad=10)
    fig.legend(plot_lines[::5], all_users, loc='upper right')
    fig.savefig('all_height_scatter.png', bbox_inches='tight')

    import pdb; pdb.set_trace()


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def remove_spikes(array_data):

    test = list(array_data) 
    array_data = np.array(array_data)
    for i in range(len(array_data)-1):
        point = array_data[i+1]
        prev_point = array_data[i]

        if prev_point < 0:
            #find first non negative value
            idx = np.where(array_data > 0)[0]
            if len(idx > 0):
                idx = idx[0]
                #test[i+1] = array_data[idx]
                array_data[i] = array_data[idx]

        point = array_data[i+1]
        prev_point = array_data[i]

        if prev_point < 0:
            #find first non negative value
            idx = np.where(array_data > 0)[0]
            if len(idx > 0):
                idx = idx[0]
                #test[i+1] = array_data[idx]
                array_data[i] = array_data[idx]

        #print abs(point-prev_point)
        if abs(point - prev_point) > 0.1:
            array_data[i+1] = prev_point
            #test[i+1] = prev_point

    return array_data

def use_last_good(a):

    last_good = 0.0
    for i in range(len(a)-1):
        point = a[i+1]
        prev_point = a[i]

        #print abs(point-prev_point)
        if abs(point - prev_point) > 0.01:
            #a[i+1] = prev_point
            a[i] = last_good
        else:
            print last_good
            last_good = point

    return a

def main():

    if len(sys.argv) == 2:
        input_file = sys.argv[1]

        load_pkl(input_file)

    else:
        print 'error'

if __name__== "__main__":
    main()

