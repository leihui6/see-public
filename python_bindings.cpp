#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "SEE.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <stdexcept>

namespace py = pybind11;

SeePointCloudPtr py_list_to_point_cloud(const py::object& points) {
    SeePointCloudPtr cloud(new pcl::PointCloud<SeePoint>);
    for (const auto& point : points) {
        if (!py::isinstance<py::sequence>(point) || py::len(point) != 3) {
            throw std::runtime_error("Each point must be a sequence of 3 numbers");
        }
        SeePoint p;
        py::iterator it = py::iter(point);
        try {
            p.x = py::cast<float>(*(it++));
            p.y = py::cast<float>(*(it++));
            p.z = py::cast<float>(*(it));
        }
        catch (const py::cast_error& e) {
            throw std::runtime_error("Failed to convert point coordinates to float");
        }
        cloud->push_back(p);
    }
    return cloud;
}

PYBIND11_MODULE(pysee, m) {
    m.doc() = "Python bindings for SEE library";

    py::class_<SEE>(m, "SEE")
        .def(py::init<std::string>(), "Initialize a SEE instance with a configuration file")
        .def("search_nbv_once", [](SEE& self, const py::list& cloud, const py::list& current_v, py::list next_v) {
        if (py::len(current_v) != 6 || py::len(next_v) != 6) {
            throw py::value_error("current_v and next_v must be lists of 6 floats");
        }
        SeePointCloudPtr cpp_cloud = py_list_to_point_cloud(cloud);
        SeeView cpp_current_v;
        cpp_current_v.x = py::cast<float>(current_v[0]);
        cpp_current_v.y = py::cast<float>(current_v[1]);
        cpp_current_v.z = py::cast<float>(current_v[2]);
        cpp_current_v.view_x = py::cast<float>(current_v[3]);
        cpp_current_v.view_y = py::cast<float>(current_v[4]);
        cpp_current_v.view_z = py::cast<float>(current_v[5]);
        SeeView cpp_next_v;
        try {
            self.SearchNBVOnce(cpp_cloud, cpp_current_v, cpp_next_v);
        }
        catch (const std::exception& e) {
            throw std::runtime_error(std::string("SEE error: ") + e.what());
        }
        // Update the next_v list in-place
        next_v[0] = cpp_next_v.x;
        next_v[1] = cpp_next_v.y;
        next_v[2] = cpp_next_v.z;
        next_v[3] = cpp_next_v.view_x;
        next_v[4] = cpp_next_v.view_y;
        next_v[5] = cpp_next_v.view_z;
            }, "Perform a single NBV search and update the next_v list in-place");

    m.def("init", [](const std::string& config_filename) {
        return SEE(config_filename);
        }, "Create and return a new SEE instance with the given configuration file");
}