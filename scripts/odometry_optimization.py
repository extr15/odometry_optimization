#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_optimization')
import rospy
import yaml
import os
import math
import numpy as np
import post_processing
from viso2_ros.msg import VisoInfo
from fovis_ros.msg import FovisInfo
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
runtime = []

def function_to_min(param, *args):
    """
    Function to be minimized. Simply launches the odometry and processes the results
    """
    global iteration_num, vect, results_table, runtime
    vect = []
    runtime = []
    param_name = args[0]
    gt_file = args[1]
    sample_step = args[2]
    cmd = args[3]
    error_to_min = args[4]
    algorithm = args[5]

    # First, set the parameters into the parameters server
    if type(param) is np.float64:
        param = np.asscalar(param)
    x = param

    # In Fovis algorithm all parameters must be set as string.
    if (algorithm == "viso2"):
        rospy.set_param(param_name, x)
    else:
        rospy.set_param(param_name, str(x))

    # Start the roslaunch process for visual odometry
    os.system(cmd)

    # When launch file finishes...
    errors = post_processing.process(np.array(vect, np.float), gt_file, sample_step)

    # Save the result
    iteration_num += 1
    results_table.append([iteration_num] + [x] + [errors[0]] + [errors[1]] + ["{:10.6f}".format(np.average(runtime, 0))])

    # Return the error to minimize
    if error_to_min == 0:
        ret = float(errors[0])
    elif error_to_min == 1:
        ret = float(errors[1])
    else:
        ret = math.sqrt(float(errors[0])*float(errors[0]) + float(errors[1])*float(errors[1]))
    return ret

def odometry_callback(data):
    """
    Processes the messages recived from odometry msg
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

def info_callback(data):
    """
    Processes the messages recived from info msg
    """
    global runtime
    runtime.append(data.runtime)

def odometry_listener(topic):
    """
    Defines the listener for the information message of viso2_ros
    """
    rospy.Subscriber(topic + "/odometry", Odometry, odometry_callback)

def info_listener(topic, algorithm):
    """
    Defines the listener for the information message of viso2_ros
    """
    if (algorithm == "viso2"):
        rospy.Subscriber(topic + "/info", VisoInfo, info_callback)
    else:
        rospy.Subscriber(topic + "/info", FovisInfo, info_callback)
    

if __name__ == "__main__":
    rospy.init_node('odometry_optimization')

    # Read parameters from yaml file (see the meaning in yaml file)
    stream = open("etc/params.yaml", 'r')
    params = yaml.load(stream)
    roslaunch_package = params['roslaunch_package']
    roslaunch_file = params['roslaunch_file']
    ros_topic = params['ros_topic']
    algorithm = params['algorithm']
    gt_file = params['gt_file']
    brute = params['brute']
    sample_step = params['sample_step']
    max_iter = params['max_iter']
    error_to_min = params['error_to_min'] 
    save_output_file = params['save_output_file'] 
    save_output_data = params['save_output_data'] 
    parameters = params['parameters']

    # Sanity check
    assert(algorithm == "viso2" or algorithm == "fovis")

    # The header for the output file
    header = [ "Iteration", "Params", "Trans. MAE", "Yaw-Rot. MAE", "Runtime" ]

    # Launch the listener to capture the odometry outputs
    odometry_listener(ros_topic)

    # Launch the listener to capture the odometry info
    info_listener(ros_topic, algorithm)

    # Build the roslaunch command to run the odometry
    cmd = "roslaunch " + roslaunch_package + " " + roslaunch_file

    for i in range(len(parameters)):
        iteration_num = 0

        # Output to file
        output = ""
        results_table = []
        output += "Optimizing parameter: " + parameters[i]['name'] + "\n"
        print "================================================="
        print "Optimizing parameter: " + parameters[i]['name']
        print "================================================="

        if (brute):
            # Brute force
            xopt = -1
            error = 999
            x = parameters[i]['min_value']
            while (x < parameters[i]['max_value'] + parameters[i]['step']):
                # Call the odometry evaluation function
                err_ret = function_to_min(x, 
                    parameters[i]['name'], 
                    gt_file, 
                    sample_step, 
                    cmd, 
                    error_to_min,
                    algorithm)

                # Check optimal value
                if (err_ret < error):
                    error = err_ret
                    xopt = x
                x += parameters[i]['step']

        else:
            # Launch the optimization function 
            xopt = fminbound(function_to_min, 
                parameters[i]['min_value'], 
                parameters[i]['max_value'], 
                (parameters[i]['name'], gt_file, sample_step, cmd, error_to_min, algorithm), 
                1e-05, 
                max_iter, 
                False, 
                3)

        # When the optimization for this parameter finishes, set it to default value again
        if (algorithm == "viso2"):
            rospy.set_param(parameters[i]['name'], parameters[i]['default'])
        else:
            rospy.set_param(parameters[i]['name'], str(parameters[i]['default']))

         # If user specified a directory to save the optimization data
        if (save_output_data != ""):
            param_full_name = parameters[i]['name'].split("/");
            with open(save_output_data + param_full_name[-1] + ".csv", 'w') as outfile:
                for row in results_table:
                    line = "";
                    for n in range(len(row)):
                        line += str(row[n]).replace(" ", "") + ";"
                    line = line[:-1]
                    outfile.write(line + "\n")

        # Build the results table
        rows = [header] + results_table
        results_table.append(["-> BEST <-"] + [np.asscalar(np.round(xopt))] + ["---"] + ["---"] + ["---"])
        output += utils.toRSTtable([header] + results_table) + "\n"

        # If user specified a file, save the results
        if (save_output_file != ""):
            with open(save_output_file, 'a+') as outfile:
                outfile.write(output)
