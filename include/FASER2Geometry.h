#pragma once

// std
#include <string>
#include <vector>

// Acts
#include <Acts/Geometry/CuboidVolumeBuilder.hpp>
#include <Acts/Geometry/TrackingGeometry.hpp>
#include <Acts/Geometry/TrackingGeometryBuilder.hpp>
#include <Acts/Surfaces/DiamondBounds.hpp>
#include <Acts/Surfaces/PlaneSurface.hpp>

#include "Acts/Definitions/Units.hpp"
#include "Acts/Material/HomogeneousSurfaceMaterial.hpp"
#include "Acts/Material/HomogeneousVolumeMaterial.hpp"
#include "Acts/Geometry/GeometryContext.hpp"

#include "Acts/Geometry/CylinderVolumeHelper.hpp"
#include "Acts/Geometry/KDTreeTrackingGeometryBuilder.hpp"
#include "Acts/Geometry/LayerArrayCreator.hpp"
#include "Acts/Geometry/LayerCreator.hpp"
#include "Acts/Geometry/SurfaceArrayCreator.hpp"
#include "Acts/Geometry/TrackingGeometry.hpp"
#include "Acts/Geometry/TrackingVolumeArrayCreator.hpp"
#include "Acts/Plugins/Geant4/Geant4DetectorElement.hpp"
#include "Acts/Surfaces/Surface.hpp"
#include "Acts/Plugins/Geant4/Geant4DetectorSurfaceFactory.hpp"
#include "Acts/Geometry/GeometryIdentifier.hpp"
#include "Acts/Geometry/TrackingGeometryVisitor.hpp"
#include "Acts/MagneticField/MagneticFieldProvider.hpp"


#include <memory>
#include <ostream>
#include <stdexcept>


// Visualization
#include <Acts/Visualization/GeometryView3D.hpp>
#include <Acts/Visualization/ObjVisualization3D.hpp>
#include <Acts/Visualization/ViewConfig.hpp>

// G4
#include <G4RunManager.hh>
#include <G4UIsession.hh>
#include <G4Box.hh>
#include <G4GDMLParser.hh>
#include <G4LogicalVolume.hh>
#include <G4Material.hh>
#include <G4Polyhedra.hh>
#include <G4Types.hh>
#include <G4VPhysicalVolume.hh>
#include <boost/filesystem.hpp>

class FASER2Geometry {

    public:
        FASER2Geometry(const std::string& gdmlFilem, int axisDirection=2);

        G4VPhysicalVolume* findDaughterByName(G4VPhysicalVolume* pvol, G4String name) const;

        std::shared_ptr<const Acts::TrackingGeometry> getTrackingGeometry() const
        {
            return m_trackingGeometry;
        }

        std::shared_ptr<const Acts::MagneticFieldProvider> createMagneticField(const Acts::Vector3& field);

        Acts::Vector3 getTranslation()
        {
            Acts::Vector3 acts_translation{m_global_translation.x(), m_global_translation.y(), m_global_translation.z()};
            // Acts::Vector3 acts_translation{m_translation.x(), m_translation.y(), m_translation.z()};
            return acts_translation; //m_rotation * acts_translation;
        }

    private:
        std::string m_gdmlFile;
        Acts::AxisDirection m_axisDirection; // 0 = X, 1 = Y, 2 = Z

        G4VPhysicalVolume* m_worldPhysVol{nullptr};
        G4VPhysicalVolume* m_hallPhysVol{nullptr};
        G4VPhysicalVolume* m_FASER2PhysVol{nullptr};

        G4ThreeVector m_translation;
        G4ThreeVector m_global_translation;
        Acts::RotationMatrix3 m_rotation;
        
        std::vector<std::shared_ptr<const Acts::DetectorElementBase>> m_detectorStore;
        std::vector<std::shared_ptr<const Acts::MagneticFieldProvider>> m_magFieldStore;

        std::shared_ptr<const Acts::TrackingGeometry> m_trackingGeometry{nullptr};
        Acts::GeometryContext m_geometryContext;

        G4Transform3D m_g4ToWorldTransform;

        void createGeometry();
        std::tuple<std::vector<std::shared_ptr<Acts::Surface>>, std::vector<std::shared_ptr<Acts::Geant4DetectorElement>>> buildGeant4Volumes();

        std::unique_ptr<const Acts::Logger> m_logger = Acts::getDefaultLogger("Geant4Detector", Acts::Logging::INFO);
        const Acts::Logger& logger() const
        {
            return *m_logger;
        }


};
