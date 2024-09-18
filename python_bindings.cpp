#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "SEE.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

py::dict see_view_to_dict(const SeeView& view) {
	py::dict result;
	result["position"] = py::make_tuple(view.x, view.y, view.z);
	result["orientation"] = py::make_tuple(view.view_x, view.view_y, view.view_z);
	return result;
}

PYBIND11_MODULE(pysee, m) {
	m.doc() = "Python bindings for SEE library";

	py::class_<SEE>(m, "SEE")
		.def(py::init<std::string>())
		.def("search_nbv_once", [](SEE& self, const py::list& cloud, const py::list& current_v) {
		SeePointCloudPtr cpp_cloud = py_list_to_point_cloud(cloud);

		SeeView cpp_current_v;
		cpp_current_v.x = py::cast<float>(current_v[0]);
		cpp_current_v.y = py::cast<float>(current_v[1]);
		cpp_current_v.z = py::cast<float>(current_v[2]);
		cpp_current_v.view_x = py::cast<float>(current_v[3]);
		cpp_current_v.view_y = py::cast<float>(current_v[4]);
		cpp_current_v.view_z = py::cast<float>(current_v[5]);

		SeeView cpp_nbv;
		self.SearchNBVOnce(cpp_cloud, cpp_current_v, cpp_nbv);

		return see_view_to_dict(cpp_nbv);
			});
}
