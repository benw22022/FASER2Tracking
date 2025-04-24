# FASER2 Tracking Framework

Work in progress implementation of ACTS - code probably doesn't work! Expect *extremely* janky code

Code provides a python binding of the FASER2 geometry using a GDML file from GEANT4

To compile the FASER2 geometry do:

```bash
source /cvmfs/sft.cern.ch/lcg/views/LCG_105/x86_64-el9-gcc11-opt/setup.sh
mkdir build
cd build
cmake ../FASER2Tracking
make -j 6 # Be cautious with the core option. This is very memory intensive!
```

Then to make the bindings available do:

```bash
source build/python/setup.sh                # For the ACTS bindings
export PYTHONPATH=build/lib64:$PYTHONPATH   # For the FASER2 geometry
```
