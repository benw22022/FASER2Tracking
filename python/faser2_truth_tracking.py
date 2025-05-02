#!/usr/bin/env python3

from pathlib import Path
from typing import Optional

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


def runDigitizationConfig(
    trackingGeometry,
    input: Path,
    output: Path,
):
    inputConfig = readDigiConfigFromJson(str(input))

    digiConfigurator = DigitizationConfigurator()
    digiConfigurator.compactify = True
    digiConfigurator.inputDigiComponents = inputConfig

    trackingGeometry.visitSurfaces(digiConfigurator)

    outputConfig = DigiConfigContainer(digiConfigurator.outputDigiComponents)

    writeDigiConfigToJson(outputConfig, str(output))
    
    return trackingGeometry


def runTruthTrackingKalman(
    trackingGeometry: acts.TrackingGeometry,
    field: acts.MagneticFieldProvider,
    digiConfigFile: Path,
    outputDir: Path,
    inputParticlePath: Optional[Path] = None,
    inputHitsPath: Optional[Path] = None,
    decorators=[],
    reverseFilteringMomThreshold=0 * u.GeV,
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
    )

    s = s or acts.examples.Sequencer(
        events=1000, numThreads=-1, logLevel=acts.logging.DEBUG
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
            EtaConfig(-1, 1, uniform=True), # Give a bit of variation in eta 
            MomentumConfig(1.0 * u.GeV, 100.0 * u.GeV, transverse=True),
            PhiConfig(90 * u.degree, 90 * u.degree), # Fire the muons straight along the y axis
            vtxGen=acts.examples.GaussianVertexGenerator(
                mean=acts.Vector4(0, 9400, 0, 0),
                stddev=acts.Vector4(0, 0, 0, 0),
            ),
            multiplicity=1,
            rnd=rnd,
            logLevel=acts.logging.DEBUG,
            printParticles = True,
        )
    else:
        logger.info("Reading particles from %s", inputParticlePath.resolve())
        assert inputParticlePath.exists()
        s.addReader(
            acts.examples.RootParticleReader(
                level=acts.logging.INFO,
                filePath=str(inputParticlePath.resolve()),
                outputParticles="particles_generated",
            )
        )
        s.addWhiteboardAlias("particles", "particles_generated")

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
        s.addReader(
            acts.examples.RootSimHitReader(
                level=acts.logging.INFO,
                filePath=str(inputHitsPath.resolve()),
                outputSimHits="simhits",
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
        logLevel=acts.logging.DEBUG,
    )

    addDigiParticleSelection(
        s,
        ParticleSelectorConfig(
            pt=(0.9 * u.GeV, None),
            measurements=(0, None),
            removeNeutral=True,
            removeSecondaries=True,
        ),
        logLevel=acts.logging.DEBUG,
    )

    addSeeding(
        s,
        trackingGeometry,
        field,
        rnd=rnd,
        inputParticles="particles_generated",
        seedingAlgorithm=SeedingAlgorithm.TruthSmeared,
        particleHypothesis=acts.ParticleHypothesis.muon,
    )

    addKalmanTracks(
        s,
        trackingGeometry,
        field,
        reverseFilteringMomThreshold,
    )

    s.addAlgorithm(
        acts.examples.TrackSelectorAlgorithm(
            level=acts.logging.INFO,
            inputTracks="tracks",
            outputTracks="selected-tracks",
            selectorConfig=acts.TrackSelector.Config(
                minMeasurements=0,
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


if "__main__" == __name__:
    srcdir = Path(__file__).resolve().parent.parent.parent.parent

    print(srcdir)
    detector = Tracking.FASER2Geometry("share/gdml/FASER2_only.gdml")
    trackingGeometry = detector.getTrackingGeometry()
    
    # TODO: implement the correct restricted magnetic field
    field = acts.ConstantBField(acts.Vector3(0, 0, 1 * u.T))
    

    runTruthTrackingKalman(
        trackingGeometry=trackingGeometry,
        field=field,
        digiConfigFile="_deps/acts-src/Examples/Algorithms/Digitization/share/default-smearing-config-telescope.json",
        outputDir=Path.cwd(),
    ).run()
    