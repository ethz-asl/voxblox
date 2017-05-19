from functools import partial
import numpy as np
from operator import itemgetter
import os
import subprocess
import yaml

def RunCommand(command):
  command = "".join(command)
  print "[RUNNING COMMAND]: ", command
  p = subprocess.Popen(command, shell=True, executable='/bin/bash')
  stdout, stderr = p.communicate()
  return (stdout, p.returncode)

def RunCommandInWorkspace(workspace, command):
  if isinstance(command, list):
    command = "".join(command)
  init_script = os.path.join(workspace, "devel/setup.sh")
  return RunCommand("source " + init_script + "; " + command)

def RunCommandInWorkingDirectory(working_dir, workspace, command):
  assert os.path.isdir(working_dir)
  if isinstance(command, list):
    command = "".join(command)
  chdir_cmd = "cd " + working_dir + ";"
  return RunCommandInWorkspace(workspace, chdir_cmd + command)

def GetBuildSpaceForPackage(workspace, package_name):
  package_build_path = os.path.join(workspace, "build") 
  package_build_path = os.path.join(package_build_path, package_name)
  return package_build_path
    
def RunAllBenchmarksOfPackage(workspace, package_name):
  package_build_path = GetBuildSpaceForPackage(workspace, package_name)
  command = "cd " + package_build_path + "; make run_benchmarks"
  
  init_script = os.path.join(workspace, "devel/setup.sh")
  return RunCommand("source " + init_script + "; " + command)

def GetAllBenchmarkingResultsOfPackage(workspace, package):
  package_build_space = GetBuildSpaceForPackage(workspace, package)
    
  benchmarking_result_files = list()
  for root, dirnames, filenames in os.walk(package_build_space, followlinks=True):
    for filename in filenames:
      kResultPrefix = "benchmark-"
      if filename.startswith(kResultPrefix):
        full_path = os.path.abspath(root)
        benchmarking_result_files.append(os.path.join(full_path, filename))
  return list(set(benchmarking_result_files))
  
def UnitToScaler(unit_string):
    if unit_string == "ns":
        return 1.0e-9
    elif unit_string == "us":
        return 1.0e-6
    elif unit_string == "ms":
        return 1.0e-3
    elif unit_string == "s":
        return 1.0
    assert(false, "Unhandled unit: " + unit_string)