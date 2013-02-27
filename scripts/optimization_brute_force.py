#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_optimization')
import rospy
import yaml
import os
import math
import numpy as np
import itertools as it
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

def brute_force(parameters, gt_file, sample_step, cmd, error_to_min, algorithm, save_output_data, save_output_file):
    '''
    Function to launch the brute force algorithm.
    '''

    # Compute the table of parameters with all posible combinations
    e_str = ""
    for k in range(len(parameters)):
        iterator = ("np.arange(parameters[" + str(k) + 
            "]['min_value'], parameters[" + str(k) + 
            "]['max_value'] + parameters[" + str(k) + 
            "]['step'], parameters[" + str(k) + "]['step'])")
        e_str += "range(len(" + iterator + ")),"
    e_str = e_str[:-1]

    # Initialize the evaluation parameters        
    xopt = -1
    error = 999

    # Lauch a simulation for every subset of parameters
    count = 0;
    for subset in eval("it.product(" + e_str + ")"):

        # Set the parameters into the server
        params_str = ""
        for i in range(len(subset)):
            param_vector = np.arange(
                parameters[i]['min_value'], 
                parameters[i]['max_value'] + parameters[i]['step'], 
                parameters[i]['step'])
            param_value = param_vector[subset[i]]
            set_ros_parameter(parameters[i]['name'], param_value, algorithm)
            params_str += str(param_value) + "|"

        params_str = params_str[:-1]

        print "================================================="
        print "Optimizing subset: [ " + params_str + " ]"
        print "================================================="

        # Then launch the simulation
        err_ret = launch_simulation(cmd, str(params_str), gt_file, sample_step, error_to_min)

        # Save data
        if (save_output_data != ""):
            with open(save_output_data + "brute-force.csv", 'a+') as outfile:
                row = results_table[-1]
                line = "";
                for n in range(len(row)):
                    line += str(row[n]).replace(" ", "") + ";"
                line = line[:-1]
                outfile.write(line + "\n")

        # Check optimal value
        if (err_ret < error):
            error = err_ret
            xopt = params_str
        count = count + 1;

    # Save the report when finishes
    save_output(save_output_file, "brute-force", xopt)

def launch_simulation(cmd, param_value, gt_file, sample_step, error_to_min):
    """
    Function to be minimized. Simply launches the odometry and processes the results
    """
    global iteration_num, vect, results_table, runtime
    vect = []
    runtime = []

    # Start the roslaunch process for visual odometry
    os.system(cmd)

    # When launch file finishes...
    if (len(vect) > 0):
        errors = post_processing.process(np.array(vect, np.float), gt_file, sample_step)
    else:
        errors = [np.nan, np.nan]

    # Save the result
    iteration_num += 1
    results_table.append([iteration_num] + [param_value] + [errors[0]] + [errors[1]] + ["{:10.6f}".format(np.average(runtime, 0))])

    # Return the error to minimize
    if error_to_min == 0:
        ret = float(errors[0])
    elif error_to_min == 1:
        ret = float(errors[1])
    else:
        ret = math.sqrt(float(errors[0])*float(errors[0]) + float(errors[1])*float(errors[1]))
    return ret

def save_output(save_output_file, param_name, best_value):
    '''
    Function to save the results into files
    '''
    global results_table

    # The header for the output file
    header = [ "Iteration", "Params", "Trans. MAE", "Yaw-Rot. MAE", "Runtime" ]
    output = "Optimizing parameter: " + param_name + "\n"

    # If user specified a file, save the results
    if (save_output_file != ""):

        # Build the results table
        rows = [header] + results_table
        results_table.append(["-> BEST <-"] + [best_value] + ["---"] + ["---"] + ["---"])
        output += utils.toRSTtable([header] + results_table) + "\n"

        # Write the file
        with open(save_output_file, 'a+') as outfile:
            outfile.write(output)

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

def set_ros_parameter(param_name, param_value, algorithm):
    '''
    Function to set a parameter into the ros parameter server.
    '''
    # First, set the parameters into the parameters server
    if type(param_value) is np.float64 or type(param_value) is np.int32:
        param_value = np.asscalar(param_value)

    # In Fovis algorithm all parameters must be set as string.
    if (algorithm == "viso2"):
        rospy.set_param(param_name, param_value)
    else:
        rospy.set_param(param_name, str(param_value))
    

if __name__ == "__main__":
    rospy.init_node('optimization_brute_force')

    # Read parameters from yaml file (see the meaning in yaml file)
    stream = open("etc/params.yaml", 'r')
    params = yaml.load(stream)
    roslaunch_package = params['roslaunch_package']
    roslaunch_file = params['roslaunch_file']
    ros_topic = params['ros_topic']
    algorithm = params['algorithm']
    gt_file = params['gt_file']
    sample_step = params['sample_step']
    max_iter = params['max_iter']
    error_to_min = params['error_to_min'] 
    save_output_file = params['save_output_file'] 
    save_output_data = params['save_output_data'] 
    parameters = params['parameters']

    # Sanity check
    assert(algorithm == "viso2" or algorithm == "fovis")

    # Check if files exists
    if (save_output_file != ""):
        try:
            with open(save_output_file) as f:
                os.rename(save_output_file, save_output_file + ".bk")
        except IOError as e:
            print 'No report file'
    if (save_output_data != ""):
        try:
            with open(save_output_data + "brute-force.csv") as f:
                os.rename(save_output_data + "brute-force.csv", save_output_data + "brute-force.csv" + ".bk")
        except IOError as e:
            print 'No data file'

    # Launch the listener to capture the odometry outputs
    odometry_listener(ros_topic)

    # Launch the listener to capture the odometry info
    info_listener(ros_topic, algorithm)

    # Build the roslaunch command to run the odometry
    cmd = "roslaunch " + roslaunch_package + " " + roslaunch_file

    # Brute force
    brute_force(
        parameters, 
        gt_file, 
        sample_step, 
        cmd, 
        error_to_min, 
        algorithm, 
        save_output_data, 
        save_output_file)
        
