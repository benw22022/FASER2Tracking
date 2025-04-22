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
        FASER2Geometry(const std::string& gdmlFile);

        // std::shared_ptr<const Acts::TrackingGeometry> createGeometry();

        G4VPhysicalVolume* findDaughterByName(G4VPhysicalVolume* pvol, G4String name);

    private:
        std::string m_gdmlFile;
        G4VPhysicalVolume* m_worldPhysVol{nullptr};
        G4VPhysicalVolume* m_hallPhysVol{nullptr};
        G4VPhysicalVolume* m_FASER2PhysVol{nullptr};

        std::vector<G4VPhysicalVolume*> m_trackingPhysVolumes;
        std::shared_ptr<const Acts::TrackingGeometry> m_trackingGeometry{nullptr};
        Acts::GeometryContext m_geometryContext;

        void createGeometry();


};
