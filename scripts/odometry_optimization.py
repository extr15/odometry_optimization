#!/usr/bin/env python

import roslib; 
roslib.load_manifest('odometry_optimization')
import rospy
from viso2_ros.msg import VisoInfo
import os
from multiprocessing import Process, Lock
import numpy as np
from scipy.optimize import fminbound
import utils

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

    # First, set the parameter into the parameters server
    x0 = np.asscalar(np.round(params))
    rospy.set_param('/stereo_odometer/nms_tau', x0)

    # Start the roslaunch process for visual odometry
    p = Process(target=roslaunch, args=(ros_package, launch_file))
    p.start()
    p.join()

    # When launch file finishes...
    ret = 1/np.mean(vect)

    # Save the result
    iteration_num += 1
    results_table.append([iteration_num] + [x0] + [np.mean(vect)])
    return ret

def roslaunch(ros_package, launch_file):
    """
    Launches the odometry evaluation process
    """
    cmd = "roslaunch " + ros_package + " " + launch_file
    os.system(cmd)

def callback(data):
    """
    Processes the messages recived from viso2_ros
    """
    global vect
    vect.append(data.num_inliers)

def listener(topic):
    """
    Defines the listener for the information message of viso2_ros
    """
    rospy.Subscriber(topic, VisoInfo, callback)

if __name__ == "__main__":
    rospy.init_node('odometry_optimization')

    # Parameters
    ros_package = "odometry_evaluation"
    launch_file = "odometry_evaluation.launch"
    ros_topic = "stereo_odometer/info"

    # Launch the listener to capture the odometry outputs
    listener(ros_topic)

    # Maximum number of function evaluations
    max_iter = 10

    # Init the parameters to be optimized and go go go
    param_bounds = [40, 60]
    xopt = fminbound(function_to_min, param_bounds[0], param_bounds[1], (), 1e-05, max_iter, False, 3)

    # Show the result
    header = [ "Iteration", "Params", "Function Value" ]
    results_table.append(["-> BEST <-"] + [xopt] + ["---"])
    utils.toRSTtable([header] + results_table)


    
