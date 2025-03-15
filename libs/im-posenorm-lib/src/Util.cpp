#include "Util.h"

#include <glm/glm.hpp>

#include <cmath>

namespace IMPoseNorm {

	bool IsNaN(
			const glm::dvec3& value) {

		return std::isnan(value.x)
			|| std::isnan(value.y)
			|| std::isnan(value.z);
	}
}