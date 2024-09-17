// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef SEE_PARAMS_H
#define SEE_PARAMS_H

//#include <ros/ros.h>

#include "see_core/inc/see_structs.h"

//using namespace ros::param;

//! Load SEE parameters from ROS
/*!
  \param see Struct containing SEE parameters
*/
void LoadSeeParams(SeeParams &see) {
	see.tau = 100;
	see.rho = 1000000;
	see.r = 0.01;
	see.psi = 1;
	see.ups = 0.01;
	see.d = 0;
  //param<int>("~tau", see.tau, 100);
  //param<int>("~rho", see.rho, 1000000);
  //param<float>("~r", see.r, 0.01);
  //param<float>("~psi", see.psi, 1);
  //param<float>("~ups", see.ups, 0.01);
  //param<float>("~d", see.d, 0);
}

#endif // SEE_PARAMS_H
