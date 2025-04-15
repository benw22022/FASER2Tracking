#include <pybind11/pybind11.h>
#include "../include/MyClass.h"

namespace py = pybind11;

PYBIND11_MODULE(Tracking, m) {
    m.doc() = "Example module created with Pybind11";

    py::class_<MyClass>(m, "MyClass")
        .def(py::init())
        .def(py::init<const std::string&>())
        .def("GetName", &MyClass::GetName);
}