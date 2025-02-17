#!/usr/bin/python
#
# Plots the results from the 2D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects two optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional
#
# The files have the following format:
#   ID x y yaw_radians

import matplotlib.pyplot as plot
import numpy
import sys
from optparse import OptionParser
from IPython import display

parser = OptionParser()
parser.add_option("--initial_poses", dest="initial_poses",
                  default="init_nodes.txt", help="The filename that contains the original poses.")
parser.add_option("--optimized_poses", dest="optimized_poses",
                  default="/home/uamc/SESync/C++/build/opt_poses_.txt", help="The filename that contains the optimized poses.")
(options, args) = parser.parse_args()

while(1):
    # Read the original and optimized poses files.
    poses_original = None
    if options.initial_poses != '':
      poses_original = numpy.genfromtxt(options.initial_poses, usecols = (1, 2))

    poses_optimized = None
    if options.optimized_poses != '':
      poses_optimized = numpy.genfromtxt(options.optimized_poses, usecols = (0, 1))

    # Plots the results for the specified poses.
    plot.figure()
    if poses_original is not None:
      plot.plot(poses_original[:, 0], poses_original[:, 1], '-', label="raw",
                alpha=0.5, color="green")

    if poses_optimized is not None:
      plot.plot(poses_optimized[:, 0], (poses_optimized[:, 1]), '-', label="opt",
                alpha=0.5, color="blue")

    plot.axis('equal')
    plot.legend()
    # Show the plot and wait for the user to close.
    
    plot.show(block=False)
    plot.pause(1.0)
    plot.close()
