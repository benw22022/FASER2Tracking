#include <pybind11/pybind11.h>
#include "../include/FASER2Geometry.h"

namespace py = pybind11;

PYBIND11_MODULE(Tracking, m) {
    m.doc() = "Example module created with Pybind11";

    py::class_<FASER2Geometry>(m, "FASER2Geometry")
        .def(py::init<const std::string&, int>(), py::arg("gdmlFile"), py::arg("axis") = 2)
        .def("getTrackingGeometry", &FASER2Geometry::getTrackingGeometry)
        .def("createMagneticField", &FASER2Geometry::createMagneticField);
}