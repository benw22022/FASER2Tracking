#include "../include/FASER2Geometry.h"
#include "../include/SilentG4.h"
#include "Acts/Utilities/Logger.hpp"


// #include "ActsExamples/Geant4Detector/Geant4Detector.hpp"

#include "Acts/Geometry/CylinderVolumeHelper.hpp"
#include "Acts/Geometry/KDTreeTrackingGeometryBuilder.hpp"
#include "Acts/Geometry/LayerArrayCreator.hpp"
#include "Acts/Geometry/LayerCreator.hpp"
#include "Acts/Geometry/SurfaceArrayCreator.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Geometry/TrackingVolumeArrayCreator.hpp"
#include "Acts/Plugins/Geant4/Geant4DetectorElement.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "G4Transform3D.hh"
#include "G4VPhysicalVolume.hh"



#include "Acts/Geometry/GeometryIdentifier.hpp"

class CustomGeometryIdentifierHook : public Acts::GeometryIdentifierHook {
public:
    // Constructor, if you want to pass configuration or parameters
    CustomGeometryIdentifierHook(int layerOffset, int moduleOffset)
        : m_layerOffset(layerOffset), m_moduleOffset(moduleOffset) {}

    static int layer_num = 0;

    Acts::GeometryIdentifier decorateIdentifier(Acts::GeometryIdentifier identifier, const Acts::Surface& surface) const override {
        // Define custom rules for decorating the GeometryID
        int layerId = identifier.layer() + m_layerOffset;
        int moduleId = layerId; //identifier.module() + m_moduleOffset;
        
        Acts::GeometryIdentifier id(layer_num);
        layer_num++;

        // Combine layer and module IDs into the GeometryID
        // return Acts::GeometryIdentifier(identifier.volume(), layerId, moduleId, identifier.subID());
        return id;
    }

    // // Override the identifier method
    // Acts::GeometryID identifier(const Acts::Surface& surface, const Acts::Layer& layer,
    //                              const Acts::TrackingVolume& volume) const override {
    //     // Define custom rules for generating the GeometryID
    //     int layerId = layer.layerIndex() + m_layerOffset;
    //     int moduleId = surface.name().empty() ? 0 : std::stoi(surface.name());
    //     int volumeId = volume.volumeID();

    //     // Combine layer, module, and volume IDs into the GeometryID
    //     return Acts::GeometryID(volumeId, layerId, moduleId, 0); // Last digit is surface sub-ID
    // }

private:
    int m_layerOffset;  // Offset to layer ID, for customization
    int m_moduleOffset; // Offset to module ID
};

/**
 * @brief Construct a new FASER2Geometry::FASER2Geometry object
 * copied from https://github.com/LDMX-Software/ldmx-sw/blob/trunk/Tracking/src/Tracking/geo/TrackingGeometry.cxx
 * 
 * @param gdmlFile 
 * The GDML file to use for the geometry
 */
FASER2Geometry::FASER2Geometry(const std::string& gdmlFile) : 
    m_gdmlFile{gdmlFile}
{
    
    /**
     * We are about to use the G4GDMLParser and would like to silence
     * the output from parsing the geometry. This can only be done by
     * redirecting G4cout and G4cerr via the G4UImanager.
     *
     * The Simulator (if it is running) will already do this redirection
     * for us and we don't want to override it, so we check if there is
     * a simulation running by seeing if the run manager is created. If
     * it isn't, then we redirect G4cout and G4cerr to a G4Session that
     * just throws away all those messages.
     */
    // std::unique_ptr<SilentG4> silence;
    // if (G4RunManager::GetRunManager() == nullptr) {
    //     // no run manager ==> no simulation
    //     silence = std::make_unique<SilentG4>();
    //     // these lines compied from G4UImanager::SetCoutDestination
    //     // to avoid creating G4UImanager unnecessarily
    //     G4cout.SetDestination(silence.get());
    //     G4cerr.SetDestination(silence.get());
    // }

    // Get the world volume
    G4GDMLParser parser;

    // Validation requires internet
    parser.Read(m_gdmlFile, false);

    m_worldPhysVol = parser.GetWorldVolume();
    m_hallPhysVol = findDaughterByName(m_worldPhysVol, "hallPV");

    if (m_hallPhysVol == nullptr) {
        std::cerr << "Hall volume not found in GDML file!" << std::endl;
        throw std::runtime_error("Hall volume not found in GDML file!");
    }

    m_FASER2PhysVol = findDaughterByName(m_hallPhysVol, "FASER2Physical");
    
    if (m_FASER2PhysVol == nullptr) {
        std::cerr << "FASER2Physical volume not found in GDML file!" << std::endl;
        throw std::runtime_error("FASER2Physical volume not found in GDML file!");
    }
    // if (silence) {
    //     // we created the session and silenced G4
    //     // undo that now incase others have use for G4
    //     // nullptr => standard (std::cout and std::cerr)
    //     G4cout.SetDestination(nullptr);
    //     G4cerr.SetDestination(nullptr);
    // }

    // Now get can look through the world volume and find the tracking volumes
    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2UpstreamTrackerLayer_1"));
    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2UpstreamTrackerLayer_2"));
    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2UpstreamTrackerLayer_3"));

    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2DownstreamTrackerLayer_1"));
    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2DownstreamTrackerLayer_2"));
    m_trackingPhysVolumes.push_back(findDaughterByName(m_FASER2PhysVol, "F2DownstreamTrackerLayer_3"));

    createGeometry();
}

G4VPhysicalVolume* FASER2Geometry::findDaughterByName(G4VPhysicalVolume* pvol, G4String name) {
    G4LogicalVolume* lvol = pvol->GetLogicalVolume();
    // std::cout << "lvol->GetNoDaughters() = " << lvol->GetNoDaughters() << std::endl;
    for (G4int i = 0; i < lvol->GetNoDaughters(); i++) 
    {
        G4VPhysicalVolume* fDaughterPhysVol = lvol->GetDaughter(i);
        std::string dName = fDaughterPhysVol->GetName();
        if (dName.find(name) != std::string::npos){
            std::cout << "Found daughter volume: " << dName << std::endl;
            return fDaughterPhysVol;
        }
    }
    std::cerr << "Daughter volume: " << name << " not found!" << std::endl;
    return nullptr;
}

void FASER2Geometry::createGeometry() {
    
    // Set the geometry context
    m_geometryContext = Acts::GeometryContext();

    auto [surfaces, elements] = buildGeant4Volumes();

    // Surface array creator
    auto surfaceArrayCreator = std::make_shared<const Acts::SurfaceArrayCreator>(
        Acts::SurfaceArrayCreator::Config(),
        logger().clone("SurfaceArrayCreator"));
    // Layer Creator
    Acts::LayerCreator::Config lcConfig;
    lcConfig.surfaceArrayCreator = surfaceArrayCreator;
    auto layerCreator = std::make_shared<Acts::LayerCreator>(
        lcConfig, logger().clone("LayerCreator"));
    // Layer array creator
    Acts::LayerArrayCreator::Config lacConfig;
    auto layerArrayCreator = std::make_shared<const Acts::LayerArrayCreator>(
        lacConfig, logger().clone("LayerArrayCreator"));
    // Tracking volume array creator
    Acts::TrackingVolumeArrayCreator::Config tvacConfig;
    auto tVolumeArrayCreator =
        std::make_shared<const Acts::TrackingVolumeArrayCreator>(
            tvacConfig, logger().clone("TrackingVolumeArrayCreator"));
    // configure the cylinder volume helper
    Acts::CylinderVolumeHelper::Config cvhConfig;
    cvhConfig.layerArrayCreator = layerArrayCreator;
    cvhConfig.trackingVolumeArrayCreator = tVolumeArrayCreator;
    auto cylinderVolumeHelper =
        std::make_shared<const Acts::CylinderVolumeHelper>(
            cvhConfig, logger().clone("CylinderVolumeHelper"));

    // Configure the tracking geometry builder, copy the surfaces in
    Acts::KDTreeTrackingGeometryBuilder::Config kdtCfg;
    kdtCfg.surfaces = surfaces;
    kdtCfg.layerCreator = layerCreator;
    kdtCfg.trackingVolumeHelper = cylinderVolumeHelper;
    Acts::ProtoDetector protoDetector = Acts::ProtoDetector();
    protoDetector.name = "FASER2";
    std::shared_ptr<const Acts::GeometryIdentifierHook> geometryIdentifierHook = std::make_shared<CustomGeometryIdentifierHook>(0,0);
    // kdtCfg.protoDetector = m_cfg.protoDetector;
    // kdtCfg.geometryIdentifierHook = m_cfg.geometryIdentifierHook;

    // The KDT tracking geometry builder
    auto kdtBuilder = Acts::KDTreeTrackingGeometryBuilder(
        kdtCfg, logger().clone("KDTreeTrackingGeometryBuilder"));

    m_trackingGeometry = kdtBuilder.trackingGeometry(m_geometryContext);
    
}


std::tuple<std::vector<std::shared_ptr<Acts::Surface>>, std::vector<std::shared_ptr<Acts::Geant4DetectorElement>>> FASER2Geometry::buildGeant4Volumes() {
    
    // Generate the surface cache
    Acts::Geant4DetectorSurfaceFactory::Cache g4SurfaceCache;
    G4Transform3D g4ToWorld;
    Acts::Geant4DetectorSurfaceFactory::Options g4SurfaceOptions;
    g4SurfaceOptions.sensitiveSurfaceSelector =
        std::make_shared<Acts::Geant4PhysicalVolumeSelectors::NameSelector>(
            std::vector<std::string>{"F2UpstreamTrackerLayer_1", "F2UpstreamTrackerLayer_2", "F2UpstreamTrackerLayer_3",
                                      "F2DownstreamTrackerLayer_1", "F2DownstreamTrackerLayer_2", "F2DownstreamTrackerLayer_3"},
            false);

    g4SurfaceOptions.passiveSurfaceSelector = 
        std::make_shared<Acts::Geant4PhysicalVolumeSelectors::NameSelector>(
            std::vector<std::string>{"Magnet", "DecayVol"},
            false);

    Acts::Geant4DetectorSurfaceFactory{}.construct(g4SurfaceCache, g4ToWorld, *m_FASER2PhysVol, g4SurfaceOptions);

    std::cout << "Found " << g4SurfaceCache.matchedG4Volumes
    << " matching  Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.sensitiveSurfaces.size()
    << " converted sensitive Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.passiveSurfaces.size()
    << " converted passive Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.convertedMaterials
    << " converted Geant4 Material slabs." << std::endl;

    std::vector<std::shared_ptr<Acts::Surface>> surfaces;
    std::vector<std::shared_ptr<Acts::Geant4DetectorElement>> elements;

    // Reserve the right amount of surfaces
    surfaces.reserve(g4SurfaceCache.sensitiveSurfaces.size() +
    g4SurfaceCache.passiveSurfaces.size());
    elements.reserve(g4SurfaceCache.sensitiveSurfaces.size());

    // Add the sensitive surfaces
    unsigned int sensitiveID = 0;
    for (const auto& [e, s] : g4SurfaceCache.sensitiveSurfaces) {
    surfaces.push_back(s);
    elements.push_back(e);
    m_surfacesById[sensitiveID] = s;
    sensitiveID++;
    }

    // Visit the surfaces to build geoIDmap
    // m_trackingGeometry->visitSurfaces(surfaces);

    // Add the passive surfaces
    surfaces.insert(surfaces.end(), g4SurfaceCache.passiveSurfaces.begin(),
    g4SurfaceCache.passiveSurfaces.end());

    return {std::move(surfaces), std::move(elements)};
}
