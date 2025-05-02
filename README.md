# FASER2 Tracking Framework

## Getting started

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

Then to make the bindings available do (on every login):

```bash
source build/setup.sh
```

To run the truth tracking example do:

```bash
cd build
python FASER2Tracking/python/faser2_truth_tracking.py
```

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
