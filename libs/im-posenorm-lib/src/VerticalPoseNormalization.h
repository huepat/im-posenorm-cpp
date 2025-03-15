#pragma once

#include <im-posenorm-lib/Geometry/IShape.h>

#include <glm/glm.hpp>

#include <vector>

namespace IMPoseNorm {

    glm::dmat3 NormalizePoseVertically(
        Geometry::IShape& shape,
        const glm::dvec3& upAxis,
        const glm::dvec3& horizontalAxis,
        const glm::dvec3& centroid,
        const std::vector<double>& sizeWeights);
}