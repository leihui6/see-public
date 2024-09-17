#include "see_core/inc/see_core.h"
//#include "see_common/inc/common_structs.h"

#include <pcl/io/pcd_io.h>

using namespace ori::see::core;

void LoadSensorParams(SensorParams& sensor) {
	sensor.fps = 30;
	sensor.f_axis = 2;
	sensor.pix_x = 848;
	sensor.pix_y = 480;
	sensor.fov_x = 69.4f;
	sensor.fov_y = 42.5f;
	sensor.noise = 0.01f;
	sensor.sensor_frame = "sensor";
	sensor.world_frame = "world";
	sensor.bounds = std::vector<float>();
}

void LoadSeeParams(SeeParams& see) {
	see.tau = 100;
	see.rho = 1000000;
	see.r = 0.01;
	see.psi = 1;
	see.ups = 0.01;
	see.d = 0;
}

bool ReachedNBV(SeeView view, SeeView nbv, NBVParams nbv_p, bool& first_view) {
	float dst_off, ort_off;

	if (first_view) {
		first_view = false;
		return true;
	}

	dst_off = (view.getVector3fMap() - nbv.getVector3fMap()).norm();
	ort_off = view.getViewVector3fMap().dot(nbv.getViewVector3fMap());
	ort_off /= view.getViewVector3fMap().norm() * nbv.getViewVector3fMap().norm();
	ort_off = acos(fmin(fmax(ort_off, -1.0), 1.0));

	return dst_off < nbv_p.dst_thres && (ort_off * 180 / M_PI) < nbv_p.ort_thres;
}

void print_view(SeeView &view) {
	for (int i = 0; i < 4; i++)
		cout << view.data[i] << " ";
	for (int i = 0; i < 3; i++)
		cout << view.view[i] << " ";
	std::cout << "\n";
}

int main(int argc, char* argv[])
{
	SeeView nbv, view;
	NBVParams nbv_params;
	SeeParams see_params;
	SensorParams sensor_params;

	LoadSeeParams(see_params);
	LoadSensorParams(sensor_params);

	AbstractCoreSPtr core(new SeeCore(see_params, sensor_params));

	bool first_view = true;
	int cloud_num = 0;

		SeePointCloudPtr cloud(new pcl::PointCloud<SeePoint>);

		if (pcl::io::loadPCDFile<SeePoint>("test.pcd", *cloud) == -1)
		{
			PCL_ERROR("Couldn't read file your_point_cloud.pcd\n");
			return -1;
		}
		// obtain current point cloud
		//cloud = getViewandCloud(cloud_out, view);
		view.x = -1.25516047;
		view.y = 0.16405614;
		view.z = 0.90033961;
		view.view_x = 0.81845733;
		view.view_y = -0.11847473;
		view.view_z = -0.56222001;

		print_view(view);

		// process the point cloud
		core->UpdatePointCloud(cloud, view);
		cout << "cloud size:" << cloud->size() << endl;
		nbv = core->GetNBV();
		
		print_view(nbv);

	return 0;
}