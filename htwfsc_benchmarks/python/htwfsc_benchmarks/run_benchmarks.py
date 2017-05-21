#!/usr/bin/env pyhton
import argparse
from collections import defaultdict
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
  plt.plot([1, 2, 3, 4])
  plt.ylabel('Bla.')
  return fig


def GeneratePerformancePlotOverParameters(benchmark_context, benchmark_data):
  if benchmark_context["cpu_scaling_enabled"]:
    print "WARNING: Dynamic frequency scaling was active during the benchmarking!"
  if benchmark_context["library_build_type"] != "release":
    print "WARNING: Benchmarking was run on a non release build."

  fig = plt.figure()
  ax = fig.add_subplot(1, 1, 1)

  # Plot stuff.
  rcParams['font.family'] = 'sans-serif'
  rcParams['font.sans-serif'] = ['Gill Sans MT']

  colors = ['#284910', '#C51929', '#ECC351']
  idx = 0
  for case_results in benchmark_data:
    case_name = str.split(str(case_results[0]["name"]), '/')[1]
    parameters = list()
    cycles = list()
    for item in case_results:
      # Radii were multiplied by 10 to avoid casting the value to int
      # while storing as JSON. Check and fix.
      parameters.append(item["radius_cm"] / 100.0)
      runtime_seconds = item["cpu_time"] * \
          helpers.UnitToScaler(item["time_unit"])
      cycles.append(runtime_seconds * benchmark_context["mhz_per_cpu"] * 1e6)
    plt.plot(parameters, cycles, marker='o', markeredgecolor='none',
             color=colors[idx], linewidth=2, markersize=6, label=case_name)
    idx += 1

  ax.set_xlabel('n')
  ax.set_ylabel('Runtime [cycles]')
  ax.set_axis_bgcolor('#E2E2E2')
  ax.yaxis.grid(True, linestyle='-', color='white')
  plt.legend(loc=0)

  benchmark_name = str.split(str(benchmark_data[0][0]["name"]), '/')[0]
  title = 'Runtime for ' + benchmark_name
  ax.set_title(title)
  return fig

# Generate a plot for each benchmark task within this report.


def GeneratePlotsForBenchmarkFile(filename):
  json_data = LoadGoogleBenchmarkJsonReportResults(filename)
  benchmark_context = json_data["context"]

  figures = list()
  results = defaultdict(list)
  for benchmark_data in json_data["benchmarks"]:
    name_string = str(benchmark_data['name'])
    split_name = str.split(name_string, '/')
    if len(split_name) > 2:
      split_name = split_name[0:2]

    benchmark, case = split_name  # str.split(name_string, '/')
    results[case].append(benchmark_data)

  results_by_case = list()
  for key, value in results.items():
    results_by_case.append(value)

  # TODO(schneith): Determine the type of benchmarking. e.g. parameter range and call
  # specialized plotting functions.
  fig = GeneratePerformancePlotOverParameters(benchmark_context, results_by_case)
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
#helpers.RunAllBenchmarksOfPackage(parsed.voxblox_workspace, "voxblox")
benchmark_files = helpers.GetAllBenchmarkingResultsOfPackage(
    parsed.voxblox_workspace, "voxblox")

# Generate a plot for each benchmark result file.
figures = list()
for benchmark_file in benchmark_files:
  figures.append(GeneratePlotsForBenchmarkFile(benchmark_file))

if parsed.show_on_screen:
  plt.show()
