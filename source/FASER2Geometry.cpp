#include "../include/FASER2Geometry.h"
#include "../include/SilentG4.h"
#include "Acts/Utilities/Logger.hpp"


// #include "ActsExamples/Geant4Detector/Geant4Detector.hpp"
#include "Acts/Plugins/Geant4/Geant4Converters.hpp"
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

// class CustomGeometryIdentifierHook : public Acts::GeometryIdentifierHook {
// public:
//     // Constructor, if you want to pass configuration or parameters
//     CustomGeometryIdentifierHook(int layerOffset, int moduleOffset)
//         : m_layerOffset(layerOffset), m_moduleOffset(moduleOffset) {}

//     // static int layer_num = 0;

//     Acts::GeometryIdentifier decorateIdentifier(Acts::GeometryIdentifier identifier, const Acts::Surface& surface) const override {
//         // Define custom rules for decorating the GeometryID
//         int layerId = identifier.layer() + m_layerOffset;
//         int moduleId = layerId; //identifier.module() + m_moduleOffset;
        
//         Acts::GeometryIdentifier id(layer_num);
//         layer_num++;

//         // Combine layer and module IDs into the GeometryID
//         // return Acts::GeometryIdentifier(identifier.volume(), layerId, moduleId, identifier.subID());
//         return id;
//     }

//     // // Override the identifier method
//     // Acts::GeometryID identifier(const Acts::Surface& surface, const Acts::Layer& layer,
//     //                              const Acts::TrackingVolume& volume) const override {
//     //     // Define custom rules for generating the GeometryID
//     //     int layerId = layer.layerIndex() + m_layerOffset;
//     //     int moduleId = surface.name().empty() ? 0 : std::stoi(surface.name());
//     //     int volumeId = volume.volumeID();

//     //     // Combine layer, module, and volume IDs into the GeometryID
//     //     return Acts::GeometryID(volumeId, layerId, moduleId, 0); // Last digit is surface sub-ID
//     // }

// private:
//     int m_layerOffset;  // Offset to layer ID, for customization
//     int m_moduleOffset; // Offset to module ID
// };

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

// This method returns the transformation to the tracker coordinates z->x x->y y->z
Acts::Transform3 FASER2Geometry::getTrackerTransform(const Acts::Transform3& trans) const 
{
    double rotationAngle = M_PI * 0.5;

    // 0 0 -1
    // 0 1 0
    // 1 0 0

    // This rotation is needed to have the plane orthogonal to the X direction.
    //  Rotation of the surfaces
    Acts::Vector3 xPos1(cos(rotationAngle), 0., sin(rotationAngle));
    Acts::Vector3 yPos1(0., 1., 0.);
    Acts::Vector3 zPos1(-sin(rotationAngle), 0., cos(rotationAngle));
    Acts::RotationMatrix3 x_rot_, y_rot_;

    y_rot_.col(0) = xPos1;
    y_rot_.col(1) = yPos1;
    y_rot_.col(2) = zPos1;

    // Rotate the sensors to put them in the proper orientation in Z
    Acts::Vector3 xPos2(1., 0., 0.);
    Acts::Vector3 yPos2(0., cos(rotationAngle), sin(rotationAngle));
    Acts::Vector3 zPos2(0., -sin(rotationAngle), cos(rotationAngle));

    x_rot_.col(0) = xPos2;
    x_rot_.col(1) = yPos2;
    x_rot_.col(2) = zPos2;

    Acts::Vector3 pos{trans.translation()(2), trans.translation()(0), trans.translation()(1)};  

    Acts::RotationMatrix3 rotation = trans.rotation();
    rotation = x_rot_ * y_rot_ * rotation;

    Acts::Translation3 translation(pos);
    Acts::Transform3 transform(translation * rotation);

    return transform;
}

void FASER2Geometry::createGeometry() {
    
    // Set the geometry context
    m_geometryContext = Acts::GeometryContext();

    G4Box* FASER2BoundingBox = dynamic_cast<G4Box*>(m_FASER2PhysVol->GetLogicalVolume()->GetSolid());
    G4Material* FASER2G4Material = m_FASER2PhysVol->GetLogicalVolume()->GetMaterial();
    auto FASER2ActsMaterial = Acts::Geant4MaterialConverter{}.material(*FASER2G4Material, 1);

    std::cout << "Making VolumeConfig" << std::endl;
    std::cout << FASER2BoundingBox->GetXHalfLength() << ", " << FASER2BoundingBox->GetYHalfLength() << ", " << FASER2BoundingBox->GetZHalfLength() << std::endl;
    Acts::CuboidVolumeBuilder cvb;
    Acts::CuboidVolumeBuilder::VolumeConfig config;
    config.position = {0, 0, 0};
    config.length = {FASER2BoundingBox->GetZHalfLength()*1.5, FASER2BoundingBox->GetZHalfLength()*1.5, FASER2BoundingBox->GetZHalfLength()*1.5};
    config.volumeMaterial = std::make_shared<Acts::HomogeneousVolumeMaterial>(FASER2ActsMaterial);

    std::shared_ptr<Acts::TrackingVolume> trackingVolume = cvb.buildVolume(m_geometryContext, config);

    auto [surfaces, elements] = buildGeant4Volumes();
    unsigned int surface_idx{0};
    for (auto surface : surfaces) {

        //* Convert G4 Coordinates to ACTS coordinates i.e. z->x x->y y->z
        // Acts::Transform3 g4_transform = surface->transform(m_geometryContext);
        // Acts::Transform3 g42acts_transform = getTrackerTransform(g4_transform);
        // surface->transform(g42acts_transform);

        Acts::GeometryIdentifier geoId = Acts::GeometryIdentifier()
                                    .setVolume(1)
                                    .setLayer(0)
                                    .setSensitive(surface_idx);

        surface->assignGeometryId(geoId);
        trackingVolume->addSurface(surface);
        surface_idx++;
    }


    m_trackingGeometry = std::make_shared<Acts::TrackingGeometry>(trackingVolume);


      // Construct the rotation
    // This assumes the direction is AxisX, AxisY or AxisZ. No reset is necessary
    // in case of AxisZ
    Acts::RotationMatrix3 rotation = Acts::RotationMatrix3::Identity();
    //TODO Implement if (binValue == Acts::AxisDirection::AxisX) {
    //TODO Implement     rotation.col(0) = Acts::Vector3(0, 0, -1);
    //TODO Implement     rotation.col(1) = Acts::Vector3(0, 1, 0);
    //TODO Implement     rotation.col(2) = Acts::Vector3(1, 0, 0);
    //TODO Implement } else if (binValue == Acts::AxisDirection::AxisY) {
    //TODO Implement     rotation.col(0) = Acts::Vector3(1, 0, 0);
    //TODO Implement     rotation.col(1) = Acts::Vector3(0, 0, -1);
    //TODO Implement     rotation.col(2) = Acts::Vector3(0, 1, 0);
    //TODO Implement }

}


// void FASER2Geometry::createGeometry() {
    
//     // Set the geometry context
//     m_geometryContext = Acts::GeometryContext();

//     auto [surfaces, elements] = buildGeant4Volumes();

//     Acts::CuboidVolumeBuilder::VolumeConfig subDetVolumeConfig;

//     G4Box* FASER2BoundingBox = dynamic_cast<G4Box*>(m_FASER2PhysVol->GetLogicalVolume()->GetSolid());

//     G4Material* FASER2G4Material = m_FASER2PhysVol->GetLogicalVolume()->GetMaterial();

//     // auto FASER2ActsMaterial = Acts::Geant4MaterialConverter{}.surfaceMaterial(*FASER2G4Material, 1, 1);
//     auto FASER2ActsMaterial = Acts::Geant4MaterialConverter{}.material(*FASER2G4Material, 1);

//       // Construct the rotation
//     // This assumes the direction is AxisX, AxisY or AxisZ. No reset is necessary
//     // in case of AxisZ
//     Acts::RotationMatrix3 rotation = Acts::RotationMatrix3::Identity();
//     //TODO Implement if (binValue == Acts::AxisDirection::AxisX) {
//     //TODO Implement     rotation.col(0) = Acts::Vector3(0, 0, -1);
//     //TODO Implement     rotation.col(1) = Acts::Vector3(0, 1, 0);
//     //TODO Implement     rotation.col(2) = Acts::Vector3(1, 0, 0);
//     //TODO Implement } else if (binValue == Acts::AxisDirection::AxisY) {
//     //TODO Implement     rotation.col(0) = Acts::Vector3(1, 0, 0);
//     //TODO Implement     rotation.col(1) = Acts::Vector3(0, 0, -1);
//     //TODO Implement     rotation.col(2) = Acts::Vector3(0, 1, 0);
//     //TODO Implement }

//     std::cout << "Making LayerConfig" << std::endl;
    
//     std::vector<Acts::CuboidVolumeBuilder::LayerConfig> layerConfig;

//     for (const auto surface : surfaces) {
//         Acts::CuboidVolumeBuilder::VolumeConfig vcfg;
//         Acts::CuboidVolumeBuilder::LayerConfig lcfg;
//         lcfg.surfaces.push_back(surface);
//         const Acts::Vector3& sfcenter = surface->center(m_geometryContext);
//         std::cout << "==========================" << std::endl;
//         std::cout << "Layer Surface::Center position  (x, y, z) = (" << sfcenter.x() << ", " << sfcenter.y() << ", " << sfcenter.z() << ")" << std::endl;
//         std::vector<double> yb{500.0000000+sfcenter.y(),   -500.0000000+sfcenter.y()};
//         double ymax = *std::max_element(yb.begin(), yb.end());
//         double ymin = *std::min_element(yb.begin(), yb.end());
//         lcfg.active = true;    
//         lcfg.envelopeX = {-1500.0000000+sfcenter.x(),  1500.0000000+sfcenter.x()};
//         lcfg.envelopeY = {ymin, ymax};
//         lcfg.envelopeZ = {-50.0000000+sfcenter.z(),   50.0000000+sfcenter.z()};

//         // lcfg.envelopeX = {-1500,  1500};
//         // lcfg.envelopeY = {-500, 500 };
//         // lcfg.envelopeZ = {-50.0000000+sfcenter.z(),   50.0000000+sfcenter.z()};

//         if (ymax > ymin){
//             std::cout << "ylims should be ok"  << std::endl;
//         }
        
//         std::cout << "LayerConfig envelopeX = (" << lcfg.envelopeX[0] << ", " << lcfg.envelopeX[1] << ")" << std::endl;
//         std::cout << "LayerConfig envelopeY = (" << lcfg.envelopeY[0] << ", " << lcfg.envelopeY[1] << ")" << std::endl;
//         std::cout << "LayerConfig envelopeZ = (" << lcfg.envelopeZ[0] << ", " << lcfg.envelopeZ[1] << ")" << std::endl;

//         lcfg.binningDimension = Acts::AxisDirection::AxisZ; //TODO: This'll probably need to be changed when detector is rotated

//         layerConfig.push_back(lcfg);

//     }

//     std::cout << "Making VolumeConfig" << std::endl;
//     std::cout << FASER2BoundingBox->GetXHalfLength() << ", " << FASER2BoundingBox->GetYHalfLength() << ", " << FASER2BoundingBox->GetZHalfLength() << std::endl;
//     Acts::CuboidVolumeBuilder cvb;
//     Acts::CuboidVolumeBuilder::VolumeConfig config;
//     // config.position = {-1440.0000, -2210.0000, FASER2BoundingBox->GetZHalfLength()};
//     config.length = {FASER2BoundingBox->GetXHalfLength()*1.5, FASER2BoundingBox->GetYHalfLength()*1.5, FASER2BoundingBox->GetZHalfLength()*1.5};
//     config.layerCfg = layerConfig;
//     config.volumeMaterial = std::make_shared<Acts::HomogeneousVolumeMaterial>(FASER2ActsMaterial);

//     std::shared_ptr<Acts::TrackingVolume> trackingVolume = cvb.buildVolume(m_geometryContext, config);
//     m_trackingGeometry = std::make_shared<Acts::TrackingGeometry>(trackingVolume);
// }


std::tuple<std::vector<std::shared_ptr<Acts::Surface>>, std::vector<std::shared_ptr<Acts::Geant4DetectorElement>>> FASER2Geometry::buildGeant4Volumes() {
    
    // Generate the surface cache
    Acts::Geant4DetectorSurfaceFactory::Cache g4SurfaceCache;

    G4RotationMatrix rot;
    rot.rotateX(M_PI / 2);  // Rotates around X to shift Y → Z
    rot.rotateY(M_PI / 2);  // Rotates around Y to shift Z → X

    G4Box* FASER2BoundingBox = dynamic_cast<G4Box*>(m_FASER2PhysVol->GetLogicalVolume()->GetSolid());
    // G4ThreeVector translation(-1440.0000, -2210.0000, FASER2BoundingBox->GetZHalfLength());    
    G4ThreeVector translation(2210, FASER2BoundingBox->GetZHalfLength(), -1440);    
    // G4Transform3D g4ToWorld;
    G4Transform3D g4ToWorld(rot, translation);

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
    
    g4SurfaceOptions.convertMaterial = true;
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
    for (auto& [e, s] : g4SurfaceCache.sensitiveSurfaces) {
        s->assignDetectorElement(*e);
        surfaces.push_back(s);
        elements.push_back(e);
        
        // m_mapVisitor.visitSurface(s);
        m_surfacesById[sensitiveID] = s.get();
        sensitiveID++;
        // break;
    }

    for (const auto& s : surfaces) {
        std::cout << s->toString(m_geometryContext) << std::endl;
        auto mat = s->surfaceMaterialSharedPtr();   
        
        if (!mat)
        {
            std::cout << "Material for surface is nullptr" << std::endl;
        }
        else
        {
            std::cout << "Material is: " << *mat << std::endl;
        }

    }

    // Visit the surfaces to build geoIDmap
    // m_trackingGeometry->visitSurfaces(surfaces);

    // Add the passive surfaces
    // surfaces.insert(surfaces.end(), g4SurfaceCache.passiveSurfaces.begin(),
    // g4SurfaceCache.passiveSurfaces.end());

    return {std::move(surfaces), std::move(elements)};
}
