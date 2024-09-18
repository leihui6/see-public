#include "SEE.h"

SEE::SEE(std::string config_filename)
{
	std::ifstream file(config_filename);
	json jdata = json::parse(file);

	LoadSeeSensorParams(m_sensor_params, m_see_params, jdata);
}

void SEE::LoadSeeSensorParams(SensorParams& sensor, SeeParams& see, json& jdata)
{
	// Loading SensorParams
	m_sensor_params.fps = jdata["sensor"]["fps"];
	m_sensor_params.f_axis = jdata["sensor"]["f_axis"];
	m_sensor_params.pix_x = jdata["sensor"]["pix_x"];
	m_sensor_params.pix_y = jdata["sensor"]["pix_y"];
	m_sensor_params.fov_x = jdata["sensor"]["fov_x"];
	m_sensor_params.fov_y = jdata["sensor"]["fov_y"];
	m_sensor_params.noise = jdata["sensor"]["noise"];
	m_sensor_params.sensor_frame = jdata["sensor"]["sensor_frame"];
	m_sensor_params.world_frame = jdata["sensor"]["world_frame"];
	m_sensor_params.bounds = std::vector<float>();

	// Loading SeeParams
	m_see_params.rho = jdata["see"]["rho"];  // Target measurement density
	m_see_params.r = jdata["see"]["r"];      // resolution radius
	m_see_params.d = jdata["see"]["d"];      // view distance
	m_see_params.psi = jdata["see"]["psi"];  // occlusion search distance
	m_see_params.tau = jdata["see"]["tau"];  // Maximum views to update
	m_see_params.ups = jdata["see"]["ups"];  // visibility search distance

	m_core = boost::shared_ptr<AbstractCore>(new SeeCore(m_see_params, m_sensor_params));
}

void SEE::SearchNBVOnce(SeePointCloudPtr& cloud, SeeView & current_v, SeeView & nbv)
{
	// process the point cloud
	m_core->UpdatePointCloud(cloud, current_v);
	cout << "cloud size:" << cloud->size() << endl;
	nbv = m_core->GetNBV();
}

SEE::~SEE()
{
}