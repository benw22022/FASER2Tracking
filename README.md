# FASER2 Tracking Framework

## Getting started

Work in progress implementation of ACTS for FASER2 - Expect some janky code

Code provides a python binding of the FASER2 geometry using a GDML file from GEANT4

Code should work on any el9 machine (or in case you want to run this on your laptop, a docker container. A repo for this can be found here [`el9-cvmfs-docker`](https://github.com/benw22022/el9-cvmfs-docker))

To compile the FASER2 geometry do:
 
```bash
git clone --recursive https://github.com/benw22022/FASER2Tracking.git
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

## `faser2_truth_tracking.py` Options

### Changing the geometry

The `gdml` file can be specified with the `--geometry` (or `-g` option). By default, the code will use the `share/gdml/FASER2_only.gdml` file. This is the baseline SAMURAI magnet geometry with six tracking stations.

*Important note:* The code constructs the geometry by parsing the `gdml` file with GEANT4. The `FASER2Geometry` class is currently hardcoded to find GEANT4 physical volumes with specific names. I plan to make this for flexible, but for the moment the code is unlikely to work with any geometry file except for the one included in this repository.

### Changing the field

The field strength is configurable with the `--field` (`-f`) option. This just changes the absolute strength of the field, the field vector is configured in the code to point in the vertical direction only. The magnetic field is bounded by the magnet window physical volume in the GEANT4 geometry using an `Acts::MultiRangeBField`.

### Setting the input

The output of the [`FPFSim` GEANT4 simulation](https://github.com/benw22022/FPFSim) can be read into Acts if the `/histo/actsHits` macro option is set. In the output file you will find a `hits` tree and `particles` tree. Both trees are necessary for the truth tracking to work. This file can be passed to `faser2_truth_tracking.py` by setting the `--input` (`-i`) option. If this option is not set the default is to use a muon particle gun placed just in front of the first tracking layer. Currently, the particle gun settings are hardcoded. I plan to change this to make it more configurable.

The number of events to process can be set with the `--nevents` (`-n`) option and the number of CPU core to use can be set with the `--nthreads` (`-j`) option.

### Setting the output directory

The location of the output files can be set using the `--output` (`-o`) option (if not set then files are saved to the `cwd`).

### Other settings

The `--axis` (`-a`) setting can be used to change the orientation of the detector. By default, this is set to `1`, the enum corresponding to the y-axis in Acts. This is done so that the seeding works properly (the seeding algorithm assumes a cylindrical geometry). It is not recommended that you change this setting.

The digitization file is not currently configurable and is set to be `share/digitization/FASER2-digitization-smearing-x-z-config.json` which applies a Gaussian smearing to the hit `x` and `z` coordinates.

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
