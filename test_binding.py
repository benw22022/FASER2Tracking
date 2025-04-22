import sys
sys.path.append('/home/atreus/FASER2Tracking-build')
import Tracking
import ROOT

a = Tracking.MyClass("Bob")

print("This object is called", a.GetName())

geometry = Tracking.FASER2Geometry()

#geometry.exportGeometryToROOT("../FASER2_only.gdml")