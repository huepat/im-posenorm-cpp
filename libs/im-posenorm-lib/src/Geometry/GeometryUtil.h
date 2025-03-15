#pragma once

#include <im-posenorm-lib/Geometry/AABox.h>

#include <glm/glm.hpp>

#include <memory>
#include <vector>

namespace IMPoseNorm::Geometry {

	AABox GetBBox(
		const std::vector<glm::dvec3>& points);
}