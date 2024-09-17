#pragma once

#include <spdlog/spdlog.h>

class CustomInfo {
public:
	CustomInfo() {
		spdlog::info("CustomInfo constructor called");
	}
	~CustomInfo();
	void printInfo();
};