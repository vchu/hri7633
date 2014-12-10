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
from sklearn.decomposition import PCA
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D
from sklearn.lda import LDA
from sklearn.metrics import classification_report
from sklearn.svm import SVC
from sklearn.neighbors import KNeighborsClassifier

def load_pkl_type(filename):

    data_store = cPickle.load(open(filename, "rb" ))
    body_part_list = ['head', 'neck', 'left_shoulder', 'torso', 'left_knee', 'right_elbow', 'right_shoulder', 'right_hand', 'left_hip', 'right_hip', 'left_foot', 'left_elbow', 'left_hand', 'right_knee', 'right_foot']
    feat_store = dict()

    feats_all = []
    labels = []
    # For each datatype...
    for datatype in data_store:
        user_data = data_store[datatype]
        feat_store[datatype] = []

        for run_data in user_data:

            feats = []

            # Pull out torso
            torso_translation = run_data['torso']['translation']
            torso_rotation = run_data['torso']['translation']

            # compute sum squared diff between each joint to torso
            for body_part in body_part_list:

                if body_part == 'torso':
                    # change across time...
                    tb_trans_diff = np.mean(np.sqrt(np.square(np.diff(torso_translation))))
                else:
                    
                    # Pull out translation and rotation
                    body_part_trans = run_data[body_part]['translation']
                    body_part_rot = run_data[body_part]['rotation']

                    # Difference between torso and body part
                    torso_body_trans = body_part_trans - torso_translation
                    torso_body_rot = body_part_rot - torso_rotation

                    # change across time...
                    tb_trans_diff = np.mean(np.sqrt(np.square(np.diff(torso_body_trans))))
                    tb_rot_diff = np.mean(np.sqrt(np.square(np.diff(torso_body_rot))))

                # just care about translation for now?
                feats.append(tb_trans_diff)

            feat_store[datatype].append(feats)
            feats_all.append(feats)
            labels.append(datatype)

    return (feat_store, feats_all, labels)

def load_pkl_usr(filename):

    data_store = cPickle.load(open(filename, "rb" ))
    body_part_list = ['head', 'neck', 'left_shoulder', 'torso', 'left_knee', 'right_elbow', 'right_shoulder', 'right_hand', 'left_hip', 'right_hip', 'left_foot', 'left_elbow', 'left_hand', 'right_knee', 'right_foot']
    feat_store = dict()

    feats_all = []
    labels = []
    # For each user...
    for user in data_store:
        user_data = data_store[user]
        feat_store[user] = []

        for run_data in user_data:

            feats = []

            # Pull out torso
            torso_translation = run_data['torso']['translation']
            torso_rotation = run_data['torso']['translation']

            # Pull out X and Z values for the head
            head_translation = run_data['head']['translation']
            mu = np.mean(head_translation, axis=1)
            covar = np.cov(np.hstack((head_translation[:,0],head_translation[:,2])))
            #feats.append(np.median(head_translation[:,0]))
            #feats.append(np.median(head_translation[:,2]))
            feats.append(np.vstack((head_translation[:,0],head_translation[:,2])).T)

            '''
            # compute sum squared diff between each joint to torso
            for body_part in body_part_list:

                if body_part == 'torso':
                    # change across time...
                    tb_trans_diff = np.mean(np.sqrt(np.square(np.diff(torso_translation))))
                else:

                    # Pull out translation and rotation
                    body_part_trans = run_data[body_part]['translation']
                    body_part_rot = run_data[body_part]['rotation']

                    # Difference between torso and body part
                    torso_body_trans = body_part_trans - torso_translation
                    torso_body_rot = body_part_rot - torso_rotation

                    # change across time...
                    tb_trans_diff = np.mean(np.sqrt(np.square(np.diff(torso_body_trans))))
                    tb_rot_diff = np.mean(np.sqrt(np.square(np.diff(torso_body_rot))))

                # just care about translation for now?
                feats.append(tb_trans_diff)
            '''
            feat_store[user].append(feats)
            feats_all.append(feats)
            #labels.append(int(user.split('r')[-1]))
            labels.append([int(user.split('r')[-1])]*len(head_translation[:,0]))

    return (feat_store, feats_all, labels)


def reduce_dimensions(features, feat_store, datatype=False):

    # Fit PCA 
    pca = PCA(n_components=3)
    pca.fit(features)

    # Transform the features
    transformed_feats = pca.transform(features)

    transformed_feats = dict()
    for user in feat_store:

        feats = feat_store[user]
        transformed_feats[user] = pca.transform(feats)

    fig_num = 0 
    #colors = ['r','g','b','y','c','m','k']
    colors = cm.rainbow(np.linspace(0, 1, 12))
    lines = ["-","--","-.",":"]
    markers = ['o','v', 's','D', 'p','h','<','>','*']
    colorcycler = cycle(colors)
    linecycler = cycle(lines)
    markercycler = cycle(markers)
    fig = plt.figure(fig_num, figsize=(15,10))
    ax = fig.add_subplot(111,projection='3d')
    #ax = fig.add_subplot(111)

    if datatype == False:
        all_users = ['user01','user02','user03','user04','user05','user06','user07',
        'user08','user09','user13','user14','user15']
        plot_lines = []

        for user in all_users:
            
            runs = transformed_feats[user]
            # plot!
            #plt.scatter(runs[:,0],runs[:,1], color=next(colorcycler), marker=next(markercycler), s=50)
            create_line = ax.plot(runs[:,0],runs[:,1],runs[:,2], c=next(colorcycler), marker=next(markercycler), markersize=15, linestyle = 'None',label=user)
            plot_lines.append(create_line)

        ax.set_xlabel('Component 1')
        ax.set_ylabel('Component 2')
        ax.set_zlabel('Component 3')

        #fig.legend((plot_lines[::2]), all_users, loc='upper right')
        plt.legend(loc='upper left')
        fig.savefig('torso_joints_pca_scatter.png', bbox_inches='tight')

    else:
        all_types = ['visitor','second_encounter','lab_member']

        plot_lines = []

        for datatype in all_types:
            
            runs = transformed_feats[datatype]
            # plot!
            #plt.scatter(runs[:,0],runs[:,1], color=next(colorcycler), marker=next(markercycler), s=50)
            create_line = ax.plot(runs[:,0],runs[:,1],runs[:,2], c=next(colorcycler), marker=next(markercycler), markersize=15, linestyle = 'None',label=datatype)
            plot_lines.append(create_line)

        ax.set_xlabel('Component 1')
        ax.set_ylabel('Component 2')
        ax.set_zlabel('Component 3')

        import pdb; pdb.set_trace()
        #fig.legend((plot_lines[::2]), all_users, loc='upper right')
        plt.legend(loc='upper left')
        fig.savefig('torso_joints_pca_scatter_type.png', bbox_inches='tight')


def train_simple_learner(features, labels):

    # Try out KNN
    clf = KNeighborsClassifier(n_neighbors=100)
    clf.fit(np.hstack(features)[0], np.hstack(labels))

    # Try out simple gaussian
    import pdb; pdb.set_trace() 

    # Try out LDA
    #clf = LDA(n_components=3)
    #clf = LDA()
    #clf.fit(features, labels)
    pca = None

    '''
    # Fit PCA 
    pca = PCA(n_components=3)
    pca.fit(features)

    # Transform the features
    transformed_feats = pca.transform(features)
    clf = SVC()
    clf.fit(transformed_feats, labels)
    '''

    return clf, pca
    



def test_simple_learner(learner, features, labels, pca=None):

    if pca is not None:
        features = pca.transform(features)
    # Merge features back down
    feat = [np.median(x,axis=1) for x in features]
    import pdb; pdb.set_trace()
    Y_pred = learner.predict(np.vstack(feat))
    #Y_prob = learner.predict_proba(features)

    label_feat = [x[0] for x in labels]
    print classification_report(label_feat,Y_pred)

def main():

    if len(sys.argv) == 2:
        input_file = sys.argv[1]

        (feat_store, feats_all, labels) = load_pkl_usr(input_file)
        #reduce_dimensions(feats_all, feat_store)

    elif len(sys.argv) == 3:
        input_file = sys.argv[1]
        test_file = sys.argv[2]

        (feat_store, feats_all, labels) = load_pkl_usr(input_file)
        (test_feat_store, test_feats_all, test_labels) = load_pkl_usr(test_file)
        #reduce_dimensions(feats_all, feat_store)
        learner, pca = train_simple_learner(feats_all, labels) 
        test_simple_learner(learner, test_feats_all, test_labels, pca=pca)


    else:
        print 'error'

if __name__== "__main__":
    main()

