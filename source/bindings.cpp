#include <pybind11/pybind11.h>
#include "../include/FASER2Geometry.h"
#include "../include/RestrictedBField.h"

#include "Acts/MagneticField/MagneticFieldProvider.hpp"

namespace py = pybind11;

PYBIND11_MODULE(Tracking, m) {
    m.doc() = "Example module created with Pybind11";

    py::class_<Acts::MagneticFieldProvider, std::shared_ptr<Acts::MagneticFieldProvider>>(m, "MagneticFieldProvider", py::module_local())
    .def("getField", &Acts::MagneticFieldProvider::getField);
    
    py::class_<FASER2Geometry>(m, "FASER2Geometry")
        .def(py::init<const std::string&>())
        .def("getTrackingGeometry", &FASER2Geometry::getTrackingGeometry)
        .def("createMagneticField", &FASER2Geometry::createMagneticField);

    py::class_<RestrictedBField, Acts::MagneticFieldProvider, std::shared_ptr<RestrictedBField>>(m, "RestrictedBField")
        .def(py::init<const Acts::Vector3&, const Acts::Vector3&, const Acts::Vector3&>())
        .def("getField", &RestrictedBField::getField)
        .def("getFieldGradient", &RestrictedBField::getFieldGradient)
        .def("makeCache", &RestrictedBField::makeCache)
        .def("isInside", &RestrictedBField::isInside);
}