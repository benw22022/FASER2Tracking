#include "../include/FASER2Geometry.h"
#include "../include/SilentG4.h"


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
    // Create the geometry
    Acts::CuboidVolumeBuilder volumeBuilder;
    Acts::TrackingGeometryBuilder builder(volumeBuilder);
    m_trackingGeometry = builder.build(m_worldPhysVol, m_trackingPhysVolumes);
    
    // Set the geometry context
    m_nominalGeometryContext = Acts::GeometryContext();
}