#!/usr/bin/env python3

from pathlib import Path
from typing import Optional
import argparse

import Tracking # This is our python binding for our detector geometry

import acts
import acts.examples
from acts.examples import (
    readDigiConfigFromJson,
    DigitizationConfigurator,
    writeDigiConfigToJson,
    GenericDetector,
    DigiConfigContainer,
)

u = acts.UnitConstants 


def runTruthTrackingKalman(
    trackingGeometry: acts.TrackingGeometry,
    field: acts.MagneticFieldProvider,
    digiConfigFile: Path,
    outputDir: Path,
    inputParticlePath: Optional[Path] = None,
    inputHitsPath: Optional[Path] = None,
    decorators=[],
    reverseFilteringMomThreshold=0 * u.GeV,
    axisDirection=1,
    nevents=1000,
    nthreads=-1,
    offset=acts.Vector3(0,0,0),
    eta=0.1,
    s: acts.examples.Sequencer = None,
):
    from acts.examples.simulation import (
        addParticleGun,
        ParticleConfig,
        EtaConfig,
        PhiConfig,
        MomentumConfig,
        addFatras,
        addDigitization,
        ParticleSelectorConfig,
        addDigiParticleSelection,
    )
    from acts.examples.reconstruction import (
        addSeeding,
        SeedingAlgorithm,
        addKalmanTracks,
        addCKFTracks,
        TrackSelectorConfig,
        CkfConfig,
    )

    s = s or acts.examples.Sequencer(
        events=nevents, numThreads=nthreads, logLevel=acts.logging.INFO
    )

    for d in decorators:
        s.addContextDecorator(d)

    rnd = acts.examples.RandomNumbers(seed=42)
    outputDir = Path(outputDir)

    logger = acts.logging.getLogger("Truth tracking example")

    if inputParticlePath is None:
        addParticleGun(
            s,
            ParticleConfig(num=1, pdg=acts.PdgParticle.eMuon, randomizeCharge=True),
            # EtaConfig(-0.00001, 0.00001, uniform=True), # Give a bit of variation in eta 
            EtaConfig(-eta, eta, uniform=True), # Give a bit of variation in eta
            MomentumConfig(1 * u.GeV, 1000 * u.GeV, transverse=True),
            PhiConfig(90 * u.degree, 90 * u.degree), # Fire the muons straight along the y axis
            vtxGen=acts.examples.GaussianVertexGenerator(
                mean=acts.Vector4(0, -500, 0, 0),  # Place muons just in front of the first tracking station
                stddev=acts.Vector4(0, 0, 0, 0),
            ),
            multiplicity=1,
            rnd=rnd,
            logLevel=acts.logging.INFO,
            # outputDirRoot="particles_root",
            printParticles = False,
        )
    else:
        logger.info("Reading particles from %s", inputParticlePath.resolve())
        assert inputParticlePath.exists()
        cfg = Tracking.RootParticleReaderConfig(
            filePath=str(inputParticlePath.resolve()),
            outputParticles="particles_generated",
            axisDirection=axisDirection,
            offset=offset,
        )
        s.addReader(
            Tracking.RootParticleReader(
                cfg,
                level=acts.logging.INFO,
            )
        )
        s.addWhiteboardAlias("particles", "particles_generated")
        s.addWhiteboardAlias("particles_generated_selected", "particles_generated")
        s.addWhiteboardAlias("particles_simulated_selected", "particles_generated")

    if inputHitsPath is None:
        addFatras(
            s,
            trackingGeometry,
            field,
            rnd=rnd,
            enableInteractions=True,
        )
    else:
        logger.info("Reading hits from %s", inputHitsPath.resolve())
        assert inputHitsPath.exists()
        cfg = Tracking.RootSimHitReaderConfig(
            filePath=str(inputHitsPath.resolve()),
            treeName='hits',
            outputSimHits="simhits",
            axisDirection=axisDirection,
            offset=offset,
            trackingGeometry=trackingGeometry
            )
        s.addReader(
            Tracking.RootSimHitReader(
                cfg,
                level=acts.logging.INFO,
            )
        )

    addDigitization(
        s,
        trackingGeometry,
        field,
        digiConfigFile=digiConfigFile,
        rnd=rnd,
        outputDirCsv="digi_hits_csv",
        outputDirRoot="digi_hits_root",
        logLevel=acts.logging.INFO,
    )

    addDigiParticleSelection(
        s,
        ParticleSelectorConfig(
            pt=(0.9 * u.GeV, None),
            measurements=(6, None),
            removeNeutral=True,
            removeSecondaries=False,
        ),
        logLevel=acts.logging.INFO,
    )

    addSeeding(
        s,
        trackingGeometry,
        field,
        rnd=rnd,
        inputParticles="particles_generated",
        seedingAlgorithm=SeedingAlgorithm.TruthSmeared,
        particleHypothesis=acts.ParticleHypothesis.muon,
        logLevel=acts.logging.INFO,
    )

    addCKFTracks(
        s,
        trackingGeometry,
        field,
        TrackSelectorConfig(
            pt=(1 * u.GeV, None),
            # absEta=(None, 3.0),
            # loc0=(-4.0 * u.mm, 4.0 * u.mm),
            nMeasurementsMin=6,
            maxHoles=0,
            maxOutliers=0,
        ),
        CkfConfig(
            chi2CutOffMeasurement=15,
            # chi2CutOffOutlier=55.0,
            numMeasurementsCutOff=6,
            seedDeduplication=False,
            stayOnSeed=False,
            maxSteps=10000
        ),
        outputDirRoot=outputDir,
        # outputDirCsv=outputDir / "csv" if outputCsv else None,
        writeTrackStates=True,
    )

    s.addAlgorithm(
        acts.examples.TrackSelectorAlgorithm(
            level=acts.logging.INFO,
            inputTracks="tracks",
            outputTracks="selected-tracks",
            selectorConfig=acts.TrackSelector.Config(
                minMeasurements=6,
            ),
        )
    )
    s.addWhiteboardAlias("tracks", "selected-tracks")

    s.addWriter(
        acts.examples.RootTrackStatesWriter(
            level=acts.logging.INFO,
            inputTracks="tracks",
            inputParticles="particles_selected",
            inputTrackParticleMatching="track_particle_matching",
            inputSimHits="simhits",
            inputMeasurementSimHitsMap="measurement_simhits_map",
            filePath=str(outputDir / "trackstates_kf.root"),
        )
    )

    s.addWriter(
        acts.examples.RootTrackSummaryWriter(
            level=acts.logging.INFO,
            inputTracks="tracks",
            inputParticles="particles_selected",
            inputTrackParticleMatching="track_particle_matching",
            filePath=str(outputDir / "tracksummary_kf.root"),
        )
    )

    s.addWriter(
        acts.examples.TrackFitterPerformanceWriter(
            level=acts.logging.INFO,
            inputTracks="tracks",
            inputParticles="particles_selected",
            inputTrackParticleMatching="track_particle_matching",
            filePath=str(outputDir / "performance_kf.root"),
        )
    )

    return s


def get_argparser():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", "-g", type=str, default="share/gdml/FASER2_only.gdml", help = "Path to gdml geometry file. Default is share/gdml/FASER2_only.gdml (6 tracking stations, SAMUARI-style magnet)")
    parser.add_argument("--axis", "-a", type=int, default=1, help = "Enum describing the orientation of the detector axis: x=0, y=1, z=2. Default is y-axis (1)")
    parser.add_argument("--field", "-f", type=float, default=1, help = "Magnetic field strength in tesla. Default is 1 tesla.")
    parser.add_argument("--input_file", "-i", type=str, default=None, help = "Input hits & particle file. If not provided particle gun will be used")
    parser.add_argument("--nevents", "-n", type=int, default=100, help = "Number of events to parse from input file or to generate with particle gun")
    parser.add_argument("--nthreads", "-j", type=int, default=-1, help = "Number of threads to use. Default is -1 which means all available threads will be used. For debugging purposes you can set this to 1 (will make reading the output easier)")
    parser.add_argument("--output_dir", "-o", type=str, default=Path.cwd(), help = "Output directory for the results. Default is the current working directory.")
    parser.add_argument("--eta", "-e", type=float, default=0.1, help = "Absolute range of the eta distribution for the particle gun. Default is 0.1")

    return parser.parse_args()


if "__main__" == __name__:
    srcdir = Path(__file__).resolve().parent.parent.parent.parent
    
    #* Get script args
    args = get_argparser()
    
    # Parse output directory
    output_dir = Path(args.output_dir)
    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
    
    # Construct the tracking geometry
    detector = Tracking.FASER2Geometry(args.geometry, axis=args.axis)
    trackingGeometry = detector.getTrackingGeometry()
    
    field_strength_vect = acts.Vector3(0, 0, 1 * u.T)
    if args.axis == 0 :
        field_strength_vect = acts.Vector3(0, 1 * u.T, 0)
    if args.axis == 2 :
        field_strength_vect = acts.Vector3(1 * u.T, 0, 0)
    
    field = detector.createMagneticField(field_strength_vect)
    
    # Get the offset of the detector so that hits and particles are placed correctly
    # The detector in the Acts representation is recentred so we need to ajust the input hits and particles accordingly
    translation = detector.getTranslation()    
    offset = acts.Vector3(translation[0], translation[1], translation[2] - 3100) # The -3100 mm is to account for z-offset of Hall Head position (can't seem to get this from the geometry directly)
    
    print("offset is ", offset)
    
    runTruthTrackingKalman(
        trackingGeometry=trackingGeometry,
        field=field,
        digiConfigFile="share/digitization/FASER2-digitization-smearing-x-z-config.json", #TODO: Make digitization respected detector orientation 
        outputDir=output_dir,
        inputParticlePath=None if args.input_file is None else Path(args.input_file),
        axisDirection=args.axis,
        inputHitsPath=None if args.input_file is None else Path(args.input_file),
        nevents=args.nevents,
        nthreads=args.nthreads,
        offset=offset,
        eta=args.eta,
    ).run()
    