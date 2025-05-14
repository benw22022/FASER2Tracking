#include <pybind11/pybind11.h>
#include <Acts/Plugins/Python/Utilities.hpp>
#include "../include/FASER2Geometry.h"
#include "../include/RootSimHitReader.h"


namespace py = pybind11;

PYBIND11_MODULE(Tracking, m) {
    m.doc() = "Example module created with Pybind11";

    py::class_<FASER2Geometry>(m, "FASER2Geometry")
        .def(py::init<const std::string&, int>(), py::arg("gdmlFile"), py::arg("axis") = 2)
        .def("getTrackingGeometry", &FASER2Geometry::getTrackingGeometry)
        .def("createMagneticField", &FASER2Geometry::createMagneticField);

    // py::class_<ActsExamples::IReader, std::shared_ptr<ActsExamples::IReader>>(m, "IReader", py::module_local())
    // .def("name", &ActsExamples::IReader::name);

    py::object IReader = (py::object) py::module_::import("acts.examples").attr("IReader");

    py::class_<RootSimHitReader::Config>(m, "RootSimHitReaderConfig", py::module_local())
        .def(py::init<std::string, std::string, std::string, int>(), 
             py::arg("outputSimHits")="simhits",
             py::arg("treeName")="hits",
             py::arg("filePath"),
             py::arg("axisDirection")=2
            );
    
    ACTS_PYTHON_DECLARE_READER(RootSimHitReader, m, "RootSimHitReader", treeName, filePath, outputSimHits);
    
}
