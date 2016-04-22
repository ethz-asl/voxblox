#!/usr/bin/env bash

# Parameters
ROOT=$1
MASTER=$2

# Get the list of changed files
cd $ROOT
DIFF=$(git diff $MASTER --name-only)
echo $DIFF