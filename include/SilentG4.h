#include "G4UIsession.hh"

/**
 * This class throws away all of the messages from Geant4
 * Copied from https://github.com/LDMX-Software/ldmx-sw/blob/trunk/Tracking/src/Tracking/geo/TrackingGeometry.cxx
 **/
class SilentG4 : public G4UIsession {
    public:
     SilentG4() = default;
     ~SilentG4() = default;
     G4UIsession* SessionStart() { return nullptr; }
     G4int ReceiveG4cout(const G4String&) { return 0; }
     G4int ReceiveG4cerr(const G4String&) { return 0; }
   };