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

def function_to_min(param, *args):
    """
    Function to be minimized. Simply launches the odometry and processes the results
    """
    global iteration_num, vect, results_table
    vect = []
    param_list = args[0]
    gt_file = args[1]
    sample_step = args[2]
    cmd = args[3]
    error_to_min = args[4]

    # First, set the parameters into the parameters server
    x = param[0]
    rospy.set_param(param_list, x)

    # Start the roslaunch process for visual odometry
    os.system(cmd)

    # When launch file finishes...
    errors = post_processing.process(np.array(vect, np.float), gt_file, sample_step)

    # Save the result
    iteration_num += 1
    results_table.append([iteration_num] + [x] + [errors[0]] + [errors[1]])
    return float(errors[error_to_min])

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
    brute = params['brute']
    sample_step = params['sample_step']
    max_iter = params['max_iter']
    error_to_min = params['error_to_min'] 
    save_output_file = params['save_output_file'] 
    param_name = params['param_name']
    param_min = params['param_min']
    param_max = params['param_max']
    param_step = params['param_step']

    # Launch the listener to capture the odometry outputs
    listener(ros_topic)

    # Build the roslaunch command to run the odometry
    cmd = "roslaunch " + roslaunch_package + " " + roslaunch_file

    for i in range(len(param_name)):

        # Output to file
        output = ""
        results_table = []
        output += "Optimizing parameter: " + param_name[i] + "\n"
        print "================================================="
        print "Optimizing parameter: " + param_name[i]
        print "================================================="

        if (brute):
            # Brute force
            xopt = -1
            error = 999
            x = param_min[i]
            while (x < param_max[i] + param_step[i]):

                # Call the odometry evaluation function
                err_ret = function_to_min([x], 
                    param_name[i], 
                    gt_file, 
                    sample_step, 
                    cmd, 
                    error_to_min)

                # Check optimal value
                if (err_ret < error):
                    error = err_ret
                    xopt = x
            x += param_step[i]

        else:
            # Launch the optimization function 
            xopt = fminbound(function_to_min, 
                param_min[i], 
                param_max[i], 
                (param_names[i], gt_file, sample_step, cmd, error_to_min), 
                1e-05, 
                max_iter, 
                False, 
                3)

        # Show the result
        header = [ "Iteration", "Params", "Trans. MAE", "Yaw-Rot. MAE" ]
        results_table.append(["-> BEST <-"] + [np.asscalar(np.round(xopt))] + ["---"] + ["---"])
        output += utils.toRSTtable([header] + results_table) + "\n"

        # If user specified a file, save the results
        if (save_output_file != ""):
            with open(save_output_file, 'a+') as outfile:
                outfile.write(output)

    
