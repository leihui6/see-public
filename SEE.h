﻿#pragma once
#include "see_core/inc/see_core.h"
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace ori::see::core;

#ifdef SEE_EXPORTS
#define SEE_API __declspec(dllexport)
#else
#define SEE_API __declspec(dllimport)
#endif


class SEE_API SEE
{
public:
    SEE(std::string config_filename);
    void SearchNBVOnce(SeePointCloudPtr& cloud, SeeView& current_v, SeeView& nbv);
    ~SEE();

private:
    AbstractCoreSPtr m_core;
    SeeParams m_see_params;
    SensorParams m_sensor_params;
    void LoadSeeSensorParams(SensorParams& sensor, SeeParams& see, json& jdata);
};