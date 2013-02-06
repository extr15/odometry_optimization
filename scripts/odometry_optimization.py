#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_optimization')
import rospy
import yaml
import os
import numpy as np
import post_processing
from viso2_ros.msg import VisoInfo
from nav_msgs.msg import Odometry
from scipy.optimize import fminbound
from array import array
from odometry_evaluation import utils

class Error(Exception):
    """ Base class for exceptions in this module. """
    pass

# Global variables
iteration_num = 0
vect = []
results_table = []

def function_to_min(params, *args):
    """
    Function to be minimized. Simply launches the odometry and processes the results
    """
    global iteration_num, vect, results_table
    vect = []
    param_list = args[0]
    gt_file = args[1]
    sample_step = args[2]
    cmd = args[3]

    # First, set the parameters into the parameters server
    x = np.asscalar(np.round(params[0]))
    rospy.set_param(param_list, x)

    # Start the roslaunch process for visual odometry
    os.system(cmd)

    # When launch file finishes...
    errors = post_processing.process(np.array(vect, np.float), gt_file, sample_step)

    # Save the result
    iteration_num += 1
    results_table.append([iteration_num] + [x] + [errors[0]] + [errors[1]])
    return float(errors[0])

def callback(data):
    """
    Processes the messages recived from viso2_ros
    """
    global vect
    # Build the row
    row = [str(data.header.stamp), str(data.pose.pose.position.x),
    str(data.pose.pose.position.y), str(data.pose.pose.position.z),
    str(data.pose.pose.orientation.x), str(data.pose.pose.orientation.y),
    str(data.pose.pose.orientation.z), str(data.pose.pose.orientation.w),
    str(data.twist.twist.linear.x), str(data.twist.twist.linear.y),
    str(data.twist.twist.linear.z), str(data.twist.twist.angular.x),
    str(data.twist.twist.angular.y), str(data.twist.twist.angular.z),
    str(data.pose.covariance[0])]
    vect.append(row)

def listener(topic):
    """
    Defines the listener for the information message of viso2_ros
    """
    rospy.Subscriber(topic, Odometry, callback)

if __name__ == "__main__":
    rospy.init_node('odometry_optimization')

    # Read parameters from yaml file (see the meaning in yaml file)
    stream = open("etc/params.yaml", 'r')
    params = yaml.load(stream)
    roslaunch_package = params['roslaunch_package']
    roslaunch_file = params['roslaunch_file']
    ros_topic = params['ros_topic']
    gt_file = params['gt_file']
    sample_step = params['sample_step']
    max_iter = params['max_iter']
    param_name = params['param_name']

    # Parameter bounds
    min_bounds = np.array([40.0])
    max_bounds = np.array([60.0])

    # Launch the listener to capture the odometry outputs
    listener(ros_topic)

    # Build the roslaunch command to run the odometry
    cmd = "roslaunch " + roslaunch_package + " " + roslaunch_file

    # Init the parameters to be optimized and go go go
    xopt = fminbound(function_to_min, min_bounds, max_bounds, (param_name, gt_file, sample_step, cmd), 1e-05, max_iter, False, 3)

    # Show the result
    header = [ "Iteration", "Params", "Trans. MAE", "Yaw-Rot. MAE" ]
    results_table.append(["-> BEST <-"] + [np.asscalar(np.round(xopt))] + ["---"] + ["---"])
    utils.toRSTtable([header] + results_table)


    
