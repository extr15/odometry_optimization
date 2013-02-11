odometry_optimization
=====================

ROS package to optimize the parameters of viso2_ros and fovis_ros visual odometers.


Prerequisites
-------

Of course, you need ROS (http://www.ros.org/) (the package was tested using Fuerte) and the following 
ROS packages to use the odometry_optimization:

* [odometry_evaluation](https://github.com/srv/odometry_evaluation). Used to launch the 
odometry (see the pre-requisites of this package).
* [nodejs_data_viewer](https://github.com/srv/nodejs_data_viewer) (optional). Used to visualize the results.


Testing
------------

1 Configure the parameter file (etc/params.yaml). You have a template for viso2 and fovis in the same directory.

2 Just run the script odometry_optimization.py.

The results will be saved in a directory called 'output' in the same path where the script is executed.
