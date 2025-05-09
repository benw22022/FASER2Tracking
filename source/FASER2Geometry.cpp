// Acts
#include "Acts/Utilities/Logger.hpp"
#include "Acts/Definitions/Algebra.hpp"
#include "Acts/Definitions/Units.hpp"
#include "Acts/Geometry/CuboidVolumeBounds.hpp"
#include "Acts/Plugins/Geant4/Geant4Converters.hpp"
#include "Acts/Geometry/CylinderVolumeHelper.hpp"
#include "Acts/Geometry/KDTreeTrackingGeometryBuilder.hpp"
#include "Acts/Geometry/LayerArrayCreator.hpp"
#include "Acts/Geometry/LayerCreator.hpp"
#include "Acts/Geometry/SurfaceArrayCreator.hpp"
#include "Acts/Geometry/PlaneLayer.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Geometry/TrackingVolumeArrayCreator.hpp"
#include "Acts/Plugins/Geant4/Geant4DetectorElement.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include <Acts/MagneticField/MultiRangeBField.hpp>
#include <Acts/MagneticField/ConstantBField.hpp>
#include "Acts/Utilities/RangeXD.hpp"


// Geant4
#include "G4Transform3D.hh"
#include "G4VPhysicalVolume.hh"

// Std
#include <algorithm>
#include <cstddef>
#include <utility>

#include "../include/FASER2Geometry.h"
#include "../include/TelescopeDetectorElement.h"
#include "../include/RestrictedBField.h"



/**
 * @brief Construct a new FASER2Geometry::FASER2Geometry object
 * 
 * @param gdmlFile 
 * The GDML file to use for the geometry
 * @param axisDirection
 * The axis direction to use for the geometry. 0 = X, 1 = Y, 2 = Z (see: https://github.com/acts-project/acts/blob/94ed291e6f7b2afd3efc2508dac6f2c5826f0cc7/Core/include/Acts/Utilities/AxisDefinitions.hpp#L18)
 */
FASER2Geometry::FASER2Geometry(const std::string& gdmlFile, int axisDirection) : 
    m_gdmlFile{gdmlFile}
{   
    // Parse the enum
    m_axisDirection = static_cast<Acts::AxisDirection>(axisDirection);

    // Get the world volume
    G4GDMLParser parser;

    // Validation requires internet
    parser.Read(m_gdmlFile, false);
    m_worldPhysVol = parser.GetWorldVolume();

    // This bit could be improved
    // We need to isolate the FASER2 volume which is contained in the Hall volume which is contained in the world volume
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

    createGeometry();
}

/**
* @brief Find a daughter volume by name
* @param pvol The parent volume
* @param name The name of the daughter volume to find
**/
G4VPhysicalVolume* FASER2Geometry::findDaughterByName(G4VPhysicalVolume* pvol, G4String name) const {
    G4LogicalVolume* lvol = pvol->GetLogicalVolume();
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
    using namespace Acts::UnitLiterals;

    G4Box* FASER2BoundingBox = dynamic_cast<G4Box*>(m_FASER2PhysVol->GetLogicalVolume()->GetSolid());
    G4Material* FASER2G4Material = m_FASER2PhysVol->GetLogicalVolume()->GetMaterial();
    auto FASER2ActsMaterial = Acts::Geant4MaterialConverter{}.material(*FASER2G4Material, 1);

    auto [surfaces, elements] = buildGeant4Volumes();

    std::size_t nLayers = surfaces.size();
    std::vector<Acts::LayerPtr> layers(nLayers);
    std::cout << "Found " << nLayers << " layers." << std::endl;

    Acts::RotationMatrix3 rotation = Acts::RotationMatrix3::Identity();
    if (m_axisDirection == Acts::AxisDirection::AxisX) {
      rotation.col(0) = Acts::Vector3(0, 0, -1);
      rotation.col(1) = Acts::Vector3(0, 1, 0);
      rotation.col(2) = Acts::Vector3(1, 0, 0);
    } else if (m_axisDirection == Acts::AxisDirection::AxisY) {
      rotation.col(0) = Acts::Vector3(1, 0, 0);
      rotation.col(1) = Acts::Vector3(0, 0, -1);
      rotation.col(2) = Acts::Vector3(0, 1, 0);
    }

    std::vector<Acts::Vector3> surfaces_positions;
    std::vector<double> offsets{0,0};
    // std::vector<double> positions{9428.5, 9938.5, 10448.5, 10958.5, 11468.5, 11978.5};
    std::vector<double> positions{};

    // Loop over the surfaces we extracted from GEANT4 
    //! For some reason we **cannot** use these surfaces directly - they don't register hits (despite having and assocaiated detector element)
    //! So we need to create new surfaces from the GEANT4 surfaces
    unsigned int surface_idx{0};
    for (auto surface_cpy : surfaces) {

        const auto pBounds = std::make_shared<const Acts::RectangleBounds>(1500, 500); //! TODO Don't hardcode this!
        Acts::Vector3 surface_position = surface_cpy->center(m_geometryContext);
        Acts::Translation3 trans(offsets[0], offsets[1], surface_position.z());
        positions.push_back(surface_position.z());
        // Acts::Translation3 trans(offsets[0], offsets[1], positions[surface_idx]);
        Acts::Transform3 trafo(rotation * trans); // This transform will be used to place the surface in the correct position and orientation

        const auto surfaceMaterial = surface_cpy->surfaceMaterialSharedPtr();

        // Create the detector element - this is the 'sensitive' object
        std::shared_ptr<ActsExamples::TelescopeDetectorElement> detElement = nullptr;
        detElement = std::make_shared<ActsExamples::TelescopeDetectorElement>(
              std::make_shared<const Acts::Transform3>(trafo), pBounds, 1._um,
              surfaceMaterial);
        
        // Get the surface from the detector element
        auto surface = detElement->surface().getSharedPtr();

        // Place the detector element in the store - this will gives the class a copy of the shared pointer so that it persists in memory
        // If we don't do this, the detector element will be deleted when it goes out of scope and we'll get a segfault
        m_detectorStore.push_back(std::move(detElement));
    
        // We need to then place the surface in a surface array - why? I dunno but it doesn't work otherwise
        std::unique_ptr<Acts::SurfaceArray> surArray(new Acts::SurfaceArray(surface));        
        
        // Now we can create the layer which conatins the surface. Layers are like 'virtual surfaces and define boundaries
        // Layers are legacy and will be superseded by 'portals' in the future. See: https://acts.readthedocs.io/en/latest/core/geometry/legacy/legacy.html
        layers[surface_idx] = Acts::PlaneLayer::create(trafo, pBounds, std::move(surArray), 1._mm);

        // Associate the layer to the surface
        auto mutableSurface = const_cast<Acts::Surface*>(surface.get());
        mutableSurface->associateLayer(*layers[surface_idx]);
        std::cout << "Placing surface " << surface_idx << " at " << mutableSurface->center(m_geometryContext) << std::endl;
        surface_idx++;
    }

    // Now we neeed to make the LaryArray from which we can create the tracking volume. 
    Acts::LayerArrayCreator::Config lacConfig;
    Acts::LayerArrayCreator layArrCreator( lacConfig, Acts::getDefaultLogger("LayerArrayCreator", Acts::Logging::INFO));
    
    Acts::LayerVector layVec;
    for (unsigned int i = 0; i < nLayers; i++) 
    {
      layVec.push_back(layers[i]);
      std::cout << "Pushing layer " << i << " to layer vector" << std::endl;
    }

    auto length = abs(positions.back() - positions.front()) + 10._m;
    std::unique_ptr<const Acts::LayerArray> layArr(layArrCreator.layerArray(
        m_geometryContext, layVec, positions.front() - 2._mm, positions.back() + 2._mm,
        Acts::BinningType::arbitrary, m_axisDirection)); // The m_axisDirection tells the LayerArray in which direction to bin the layers - This will orient the detector in the correct direction
    
    // This is the bounding box for the detector - the functions like the world volume in Geant4
    std::shared_ptr<Acts::VolumeBounds> boundsVol = std::make_shared<Acts::CuboidVolumeBounds>(FASER2BoundingBox->GetXHalfLength()*2 + 5._mm, FASER2BoundingBox->GetYHalfLength()*2 + 5._mm, length + 10._mm);
    
    Acts::Translation3 transVol(offsets[0], offsets[1], (positions.front() + positions.back()) * 0.5);
    Acts::Transform3 trafoVol(rotation * transVol);
    
    // Create the tracking volume and the tracking geometry
    // TODO: Figure out how to fill the volume with material (i.e. air)
    auto trackVolume = std::make_shared<Acts::TrackingVolume>(
            trafoVol, boundsVol, nullptr, std::move(layArr), nullptr,
            Acts::MutableTrackingVolumeVector{}, "FASER2");

    m_trackingGeometry = std::make_unique<Acts::TrackingGeometry>(trackVolume);
}

/**
 * @brief Build the Geant4 volumes from the Geant4 FASER2 Physical Volume
 * 
 * @return std::tuple<std::vector<std::shared_ptr<Acts::Surface>>, std::vector<std::shared_ptr<Acts::Geant4DetectorElement>>>
 * A tuple containing the surfaces and the detector elements
 */
std::tuple<std::vector<std::shared_ptr<Acts::Surface>>, std::vector<std::shared_ptr<Acts::Geant4DetectorElement>>> FASER2Geometry::buildGeant4Volumes() {
    
    // Generate the surface cache
    Acts::Geant4DetectorSurfaceFactory::Cache g4SurfaceCache;

    G4RotationMatrix rot;
    // rot.rotateX(M_PI / 2);  // Rotates around X to shift Y → Z
    // rot.rotateY(M_PI / 2);  // Rotates around Y to shift Z → X

    G4Box* FASER2BoundingBox = dynamic_cast<G4Box*>(m_FASER2PhysVol->GetLogicalVolume()->GetSolid());
    auto physvolTrans = m_FASER2PhysVol->GetTranslation();
    std::cout << "FASER2PhysVol initial position: " << physvolTrans.x() << " " << physvolTrans.y() << " " << physvolTrans.z() << std::endl;
    G4ThreeVector translation(-physvolTrans.x(), -physvolTrans.y(), -physvolTrans.z());
    // G4Transform3D m_g4ToWorldTransform;
    G4Transform3D m_g4ToWorldTransform(rot, translation);  // This will translate the detector phys volume to x=0, y=0. This will make placing the surfaces easier

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
    Acts::Geant4DetectorSurfaceFactory{}.construct(g4SurfaceCache, m_g4ToWorldTransform, *m_FASER2PhysVol, g4SurfaceOptions);

    std::cout << "Found " << g4SurfaceCache.matchedG4Volumes
    << " matching  Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.sensitiveSurfaces.size()
    << " converted sensitive Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.passiveSurfaces.size()
    << " converted passive Geant4 Physical volumes." << std::endl;
    std::cout << "Found " << g4SurfaceCache.convertedMaterials
    << " converted Geant4 Material slabs." << std::endl;

    std::vector<std::shared_ptr<Acts::Surface>> surfaces;
    std::vector<std::shared_ptr<Acts::Surface>> passive_surfaces;
    std::vector<std::shared_ptr<Acts::Geant4DetectorElement>> elements;

    // Reserve the right amount of surfaces
    surfaces.reserve(g4SurfaceCache.sensitiveSurfaces.size());
    passive_surfaces.reserve(g4SurfaceCache.passiveSurfaces.size());
    elements.reserve(g4SurfaceCache.sensitiveSurfaces.size());

    // Add the sensitive surfaces
    for (auto& [e, s] : g4SurfaceCache.sensitiveSurfaces) {
        s->assignDetectorElement(*e);
        surfaces.push_back(s);
        elements.push_back(e);
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
    
    for (auto& s : g4SurfaceCache.passiveSurfaces) {
        passive_surfaces.push_back(s);
        std::cout << "Passive surface: " << s->name()<< std::endl;
    }

    // Add the passive surfaces
    // surfaces.insert(surfaces.end(), g4SurfaceCache.passiveSurfaces.begin(),
    // g4SurfaceCache.passiveSurfaces.end());

    return {std::move(surfaces), std::move(elements)};
}


/**
 * @brief Create the magnetic field for the FASER2 detector
 * 
 * @param field The magnetic field to use
 * @return std::shared_ptr<const Acts::MagneticFieldProvider> 
 * A shared pointer to the magnetic field provider
 */
std::shared_ptr<const Acts::MagneticFieldProvider> FASER2Geometry::createMagneticField(const Acts::Vector3& magField) {
    
    // Get the magnetic field from the GDML file
    G4VPhysicalVolume* magPhysVol = findDaughterByName(m_FASER2PhysVol, "FASER2MagnetYokePhysical");
    if (magPhysVol == nullptr) {
        std::cerr << "FASER2Magnet volume not found in GDML file!" << std::endl;
        throw std::runtime_error("FASER2Magnet volume not found in GDML file!");
    }
    std::cout << "Found magnet volume: " << magPhysVol->GetName() << std::endl;

    // Now work out the bounds of the magnet
    G4ThreeVector magCentre = magPhysVol->GetTranslation();
    std::cout << "Magnet center: " << magCentre.x() << ", " << magCentre.y() << ", " << magCentre.z() << std::endl;

    G4VSolid* magBoundingBox = dynamic_cast<G4VSolid*>(magPhysVol->GetLogicalVolume()->GetSolid());

    const G4VSolid* magnetYoke = magBoundingBox->GetConstituentSolid(0);
    const G4VSolid* magnetWindow = magBoundingBox->GetConstituentSolid(1);

    G4ThreeVector minMagnetWindow, maxMagnetWindow, minMagnetYoke, maxMagnetYoke, minWorld, maxWorld;
    magnetYoke->BoundingLimits(minMagnetYoke, maxMagnetYoke);
    magnetWindow->BoundingLimits(minMagnetWindow, maxMagnetWindow);

    // Translate the bounds to the correct relative position
    minMagnetWindow += magCentre;
    maxMagnetWindow += magCentre;
    minMagnetYoke += magCentre;
    maxMagnetYoke += magCentre;

    // Make sure that we rotate the magnet volume to align with the axis direction of the detector
    G4RotationMatrix rotation;
    rotation.rotateAxes(G4ThreeVector(1, 0, 0), G4ThreeVector(0, 1, 0), G4ThreeVector(0, 0, 1));
    if (m_axisDirection == Acts::AxisDirection::AxisX) {
        rotation.rotateY(90.0 * CLHEP::deg);
    } else if (m_axisDirection == Acts::AxisDirection::AxisY) {
        rotation.rotateX(90.0 * CLHEP::deg);
    }

    minMagnetWindow *= rotation;
    maxMagnetWindow *= rotation;
    minMagnetYoke   *= rotation;
    maxMagnetYoke   *= rotation;

    std::cout << "magnetYoke: min(" << minMagnetWindow.x() << ", " << minMagnetWindow.y() << ", " << minMagnetWindow.z() 
            << ") max(" << maxMagnetWindow.x() << ", " << maxMagnetWindow.y() << ", " << maxMagnetWindow.z() << ")" << std::endl;

    std::cout << "MagnetWindow: min(" << minMagnetYoke.x() << ", " << minMagnetYoke.y() << ", " << minMagnetYoke.z() 
            << ") max(" << maxMagnetYoke.x() << ", " << maxMagnetYoke.y() << ", " << maxMagnetYoke.z() << ")" << std::endl;

            
    // Build a multirange magnetic field, define field within the magnet window - set to zero outside
    using BFieldRange = std::pair<Acts::RangeXD<3, double>, Acts::Vector3>;
    
    // Need to fill in the zero field for the whole volume - make this as big as possible
    minWorld = {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
    maxWorld = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    
    Acts::RangeXD<3, double> worldRange = Acts::RangeXD<3, double>(
        {minWorld.x(), minWorld.y(), minWorld.z()},
        {maxWorld.x(), maxWorld.y(), maxWorld.z()});
        
    BFieldRange worldRegion = {worldRange, {0,0,0}};

    // Get the field range for the magnet window
    Acts::RangeXD<3, double> magRange = Acts::RangeXD<3, double>(
        {minMagnetWindow.x(), minMagnetWindow.y(), minMagnetWindow.z()}, 
        {maxMagnetWindow.x(), maxMagnetWindow.y(), maxMagnetWindow.z()});

    BFieldRange magRegion = {magRange, magField};

    // Now put it all together in the multirange field. Important to note that the order of the ranges matters - the ranges at the end of the vector will take priority
    std::shared_ptr<const Acts::MagneticFieldProvider> multiField = std::shared_ptr<const Acts::MultiRangeBField>(new Acts::MultiRangeBField({worldRegion, magRegion}));

    m_magFieldStore.push_back(multiField);

    return multiField;
}