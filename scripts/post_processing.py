#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_optimization')
import rospy
import string
import pylab
import sys
import numpy as np
from odometry_evaluation import utils
from odometry_evaluation import evaluate

class Error(Exception):
    """ Base class for exceptions in this module. """
    pass

def joint_data(ground_truth, odometry):
    
    # Minimum time difference
    eps = 0.005

    # Sanity check
    assert(len(ground_truth) == len(odometry))

    data = []
    start_time = ground_truth[0][0]
    for i in range(len(ground_truth)-1):
        row = []

        # Check time concordance
        time_diff = ground_truth[i][0] - odometry[i][0]
        if (abs(time_diff) > eps):
            raise Error("ground truth and odometry differ by %f secs."
                    % time_diff)
        
        # Save time
        row.append(ground_truth[i][0] - start_time)

        # Save odometry values
        for x in utils.calc_tf_vel(ground_truth[i], ground_truth[i+1]):
            row.append(x)
        for x in utils.calc_tf_vel(odometry[i], odometry[i+1]):
            row.append(x)

        row.append(odometry[i][14])
        row.append(ground_truth[i][0])

        # Save the current row
        data.append(row)

    return data        

def process(odometry, gt_file, sample_step):
    """
    Function that takes the result of the visual odometry and compares it to
    GT in order to obtain the rotation and translation errors.
    """
    # Odometry log is in nanoseconds, so we have to convert to seconds
    odometry[:,0] = odometry[:,0]/1e9
    # Save the odometry failure as a boolean
    odometry[:,14] = odometry[:,14] > 9990

    # Load the ground truth file
    ground_truth = pylab.loadtxt(gt_file)

    # Sample matching
    ground_truth, odometry = utils.sample_equal(ground_truth, odometry, True)

    # Time correction
    ground_truth, odometry = utils.rebase(ground_truth), utils.rebase(odometry)

    # Get the samples by distance
    ground_truth, odometry = utils.sample_equal_by_distance(ground_truth, odometry, sample_step)
    print "sample_equal_by_distance: ", len(ground_truth), "GT /", len(odometry), "OD (# of failures: ", int(np.sum(np.array(odometry)[:,14])), ")"

    # Prepare data for evaluation and compute the odometry errors
    data = joint_data(ground_truth, odometry)
    errors = evaluate.evaluate(data)

    return errors




