#include <pybind11/pybind11.h>
//// #include <pybind11/stl.h>
//// #include <pybind11/stl_bind.h>
//// #include <pybind11/complex.h>
//// #include <pybind11/functional.h>
//// #include <pybind11/chrono.h>
#include <Acts/Plugins/Python/Utilities.hpp>
#include "../include/FASER2Geometry.h"
#include "../include/RootSimHitReader.h"
#include "../include/RootParticleReader.hpp"

//// PYBIND11_MAKE_OPAQUE(std::map<long unsigned int, long unsigned int>);

namespace py = pybind11;

PYBIND11_MODULE(Tracking, m) {
    m.doc() = "Example module created with Pybind11";

    py::class_<FASER2Geometry>(m, "FASER2Geometry")
        .def(py::init<const std::string&, int>(), py::arg("gdmlFile"), py::arg("axis") = 2)
        .def("getTrackingGeometry", &FASER2Geometry::getTrackingGeometry)
        .def("createMagneticField", &FASER2Geometry::createMagneticField)
        .def("getTranslation",      &FASER2Geometry::getTranslation);

    // py::class_<ActsExamples::IReader, std::shared_ptr<ActsExamples::IReader>>(m, "IReader", py::module_local())
    // .def("name", &ActsExamples::IReader::name); //! Don't do something like this!

    py::object IReader = (py::object) py::module_::import("acts.examples").attr("IReader"); //! Do this instead
    py::object TrackingGeometry = (py::object) py::module_::import("acts").attr("TrackingGeometry"); 
    /// // py::object SimBarcode = (py::object) py::module_::import("acts.examples").attr("SimBarcode"); 
    /// py::bind_map<std::map<long unsigned int, long unsigned int>>(m, "ParticleIdMap");

    py::class_<RootSimHitReader::Config>(m, "RootSimHitReaderConfig", py::module_local())
        .def(py::init<std::string, std::string, std::string, int, Acts::Vector3, std::shared_ptr<const Acts::TrackingGeometry>>(), 
            py::arg("outputSimHits")="simhits", //! Important: You must define the args in the order that they are declared in the struct
            py::arg("treeName")="hits",
            py::arg("filePath"),
            py::arg("axisDirection")=2,
            py::arg("offset")=Acts::Vector3(0,0,0),
            py::arg("trackingGeometry")=nullptr
            );

    py::class_<RootParticleReader::Config>(m, "RootParticleReaderConfig", py::module_local())
        .def(py::init<std::string, std::string, std::string, int, Acts::Vector3>(), 
             py::arg("outputParticles")="particleCollection",
             py::arg("treeName")="particles",
             py::arg("filePath"),
             py::arg("axisDirection")=2,
             py::arg("offset")=Acts::Vector3(0,0,0)
            );
            // .def_readonly("particleIdMap", &RootParticleReader::Config::particleIdMap);
    
    ACTS_PYTHON_DECLARE_READER(RootSimHitReader, m, "RootSimHitReader", treeName, filePath, outputSimHits, axisDirection, offset);
    ACTS_PYTHON_DECLARE_READER(RootParticleReader, m, "RootParticleReader", treeName, filePath, outputParticles, axisDirection, offset);
    
}
