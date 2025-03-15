#include "InUtil.h"

#include <string>
#include <vector>

namespace IMPoseNorm::IO::In {

	std::vector<std::string> Split(
			const std::string& string,
			const std::string& delimiter) {

		std::size_t last = 0;
		std::size_t next = 0;
		std::vector<std::string> tokens{};

		while ((next = string.find(delimiter, last)) != std::string::npos) {

			tokens.push_back(
				string.substr(last, next - last));

			last = next + 1;
		}

		tokens.push_back(
			string.substr(last));

		return tokens;
	}
}