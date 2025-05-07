# FASER2 Tracking Framework

## Getting started

Work in progress implementation of ACTS for FASER2 - Expect *extremely* janky code

Code provides a python binding of the FASER2 geometry using a GDML file from GEANT4

Code should work on any el9 machine (or in case you want to run this on your laptop, a docker container. A repo for this can be found here [`el9-cvmfs-docker`](https://github.com/benw22022/el9-cvmfs-docker))

To compile the FASER2 geometry do:

```bash
source /cvmfs/sft.cern.ch/lcg/views/LCG_105/x86_64-el9-gcc11-opt/setup.sh
mkdir build
cd build
cmake ../FASER2Tracking
make -j 6 # Be cautious with the core option. This is very memory intensive!
```

Then to make the bindings available do (on every login):

```bash
source build/setup.sh
```

To run the truth tracking example do:

```bash
cd build
python FASER2Tracking/python/faser2_truth_tracking.py
```

## Visualising the geometry

To export the geometry into a format which can be visualised run the `python/geometry.py` script.

This will produce a directory called `obj` which contains `.obj` files.

These files can be visualised using the [`meshlab`](https://www.meshlab.net/#download) tool.
On Linux you can install `meshlab` via the command line:

```bash
sudo apt update
sudo apt install meshlab
```

You can then simply type `meshlab` in your terminal to start the application. The next step is to load the `.obj` files by going to `file -> Import Mesh` (or use `ctrl + I`) and then selecting the `.obj` file(s) you wish to load. There should be a file for each sensitive and passive surface.

If you find a layer missing unexpectedly, double-check your geometry. It could be that your bounding volume is too small.

## Known issues

There is an issue with the tracking when firing particles with very small $\eta$. When this happens you might end up with the following warning:

```bash
15:58:22 FatrasSimula DEBUG 1 simulated particles (final state) 
15:58:22 FatrasSimula DEBUG 6 simulated hits 
15:58:22 Digitization DEBUG Loaded 6 sim hits 
15:58:22 Digitization DEBUG Starting loop over modules ...
15:58:22 ParticleSele DEBUG event 0 selected 1 from 1 particles 
15:58:22 ParticleSele DEBUG filtered out because of charge: 0 
15:58:22 ParticleSele DEBUG filtered out because of hit count: 0 
15:58:22 ParticleSele DEBUG filtered out because of measurement count: 0 
15:58:22 KalmanFitter ERROR KalmanFilter failed: KalmanFitterError:4, No measurement detected during the propagation
```

I'm not 100% sure why this happens. It appears that `FATRAS` correctly generates the hits but then the `DigiParticleSelector` discards them. Perhaps due to a conflict between the momentum config??
