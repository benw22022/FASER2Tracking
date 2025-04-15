# Setup LCG release
source /cvmfs/sft.cern.ch/lcg/views/LCG_105/x86_64-el9-gcc11-opt/setup.sh

# Setup the python bindings
if [ -n "$ZSH_VERSION" ]; then
    python_dir=${0:a:h}
elif [ -n "$BASH_VERSION" ]; then
    python_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}"  )" &> /dev/null && pwd  )
else
    # If the current shell is not ZSH or Bash, we can't guarantee that the
    # script will work, so we throw an error.
    echo "ERROR:   neither ZSH nor Bash was detected, other shells are not supported. The environment has not been modified."
    exit 1
fi

export PYTHONPATH=$python_dir:$PYTHONPATH