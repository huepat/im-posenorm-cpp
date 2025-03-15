#pragma once

#include <string>
#include <vector>

namespace IMPoseNorm::IO::In {

	std::vector<std::string> Split(
		const std::string& string,
		const std::string& delimiter);
}