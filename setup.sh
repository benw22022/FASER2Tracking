#!/bin/bash
# This script sets up the environment for FASER2 Python bindings.

# Setup the LCG release
source /cvmfs/sft.cern.ch/lcg/views/LCG_105/x86_64-el9-gcc11-opt/setup.sh

# Setup the python bindings
BUILD_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${BUILD_DIR}/python/setup.sh                      # Sourcing this script sets up the Acts python bindings
export PYTHONPATH=${BUILD_DIR}/lib64:$PYTHONPATH         # This adds our python bindings to the PYTHONPATH
echo "INFO:    Setup FASER2 Python bindings complete."