#!/usr/bin/env bash
# Copyright 2014 Google Inc. All Rights Reserved.
# Distributed under the Project Tango Preview Development Kit (PDK) Agreement.
# CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.

# Parameters
ROOT=$1

# Get the list of staged files
cd $ROOT
DIFF=$(git diff --staged --name-only)
echo $DIFF