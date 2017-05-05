#!/usr/bin/env pyhton
import argparse
from functools import partial
from itertools import repeat
import helpers as helpers
import json
from matplotlib import rcParams
import matplotlib.pyplot as plt
import numpy as np
import multiprocessing
import os
from pprint import pprint
import shutil
import sys

def LoadGoogleBenchmarkJsonReportResults(filename):
    with open(filename) as data_file:    
        data = json.load(data_file)
    return data

def PlotPerformanceOverParameter(parameter_values, performance_values):
    fig = plt.figure()
    plt.plot([1,2,3,4])
    plt.ylabel('Bla.')
    return fig
        
def GeneratePerformancePlotOverParameters(benchmark_context, benchmark_data):
    if benchmark_context["cpu_scaling_enabled"]:
        print "WARNING: Dynamic frequency scaling was active during the benchmarking!"
    if benchmark_context["library_build_type"] != "release":
        print "WARNING: Benchmarking was run on a non release build."
        
    # TODO(schneith): Get parameter range from JSON.
    parameter = 1
    
    runtime_seconds = benchmark_data["cpu_time"] * \
        helpers.UnitToScaler(benchmark_data["time_unit"])
    cycles = runtime_seconds * benchmark_context["mhz_per_cpu"] * 1e6
    
    # Plot stuff.
    rcParams['font.family'] = 'sans-serif'
    rcParams['font.sans-serif'] = ['Gill Sans MT']
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.plot(parameter, cycles, marker='o', markeredgecolor='none', color='#FF4504', linewidth=2, markersize=6)
    ax.set_xlabel('n')
    ax.set_ylabel('Runtime [cycles]')
    ax.set_axis_bgcolor('#E2E2E2')
    ax.yaxis.grid(True, linestyle='-', color='white')
    
    title = benchmark_data["name"] + " [iterations: " + str(benchmark_data["iterations"]) + "]"
    ax.set_title(title)
    return fig

# Generate a plot for each benchmark task within this report.
def GeneratePlotsForBenchmarkFile(filename):
    json_data = LoadGoogleBenchmarkJsonReportResults(filename)
    benchmark_context = json_data["context"]
    
    figures = list()
    for benchmark_data in json_data["benchmarks"]:
        #TODO(schneith): Determine the type of benchmarking. e.g. parameter range and call
        # specialized plotting functions.
        fig = GeneratePerformancePlotOverParameters(benchmark_context, benchmark_data)
        figures.append(fig)
    return figures

# Parse input arguments.
parser = argparse.ArgumentParser(description='Lifelong calibration evaluation.')
parser.add_argument('--voxblox-workspace', dest='voxblox_workspace', nargs='?', 
                    default="/home/user/code/htwfnc_ws", help='Voxblox workspace.')
parser.add_argument('--show-on-screen', dest='show_on_screen', type=bool,
                    default=True, help='Show plots on screen?')
parsed = parser.parse_args()

# Build, run the benchmarks and collect the results.
assert(os.path.isdir(parsed.voxblox_workspace))
helpers.RunAllBenchmarksOfPackage(parsed.voxblox_workspace, "voxblox")
benchmark_files = helpers.GetAllBenchmarkingResultsOfPackage(
    parsed.voxblox_workspace, "voxblox")

# Generate a plot for each benchmark result file.
figures = list()
for benchmark_file in benchmark_files:
    figures.append(GeneratePlotsForBenchmarkFile(benchmark_file))

if parsed.show_on_screen:
    plt.show()
