#pragma once

#include <im-posenorm-lib/IMPoseNorm.h>

#include <cmath>
#include <cstddef>

namespace IMPoseNorm {

    const double EPSILON = 10E-5;
    const double HORIZONTAL_PEAK_RATIO = 0.75;
    const double VERTICAL_PEAK_RATIO = 0.75;
    const double UNAMBIGUOUS_ALIGN_SIZE_FRACTION = 0.1;
    const double DEGREE_45 = DegreeToRadian(45.0);
    const double DEGREE_90 = DegreeToRadian(90.0);
    const double DEGREE_180 = DegreeToRadian(180.0);
    const double HORIZONTAL_RESOLUTION = DegreeToRadian(1.0);
    const double VERTICAL_RESOLUTION = DegreeToRadian(1.0);
    const double HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MIN_THRESHOLD = DegreeToRadian(45.0);
    const double HORIZONTAL_ALIGNMENT_VERTICAL_ANGLE_MAX_THRESHOLD = DegreeToRadian(135.0);
    const double HORIZONTAL_ANGLE_REFINEMENT_ANGLE_RADIUS = DegreeToRadian(5.0);
    const double VERTICAL_ALIGNMENT_ANGLE_RADIUS = DegreeToRadian(40.0);
    const double VERTICAL_AXIS_REFINEMENT_ANGLE_RADIUS = DegreeToRadian(5.0);
    const double VERTICAL_GRID_CLUSTERING_THRESHOLD = 2 * VERTICAL_RESOLUTION;

    const std::size_t HORIZONTAL_GRID_SIZE = static_cast<std::size_t>(
        std::ceil(DEGREE_90 / HORIZONTAL_RESOLUTION));

    const std::size_t VERTICAL_GRID_SIZE_0 = static_cast<std::size_t>(
        std::ceil(DEGREE_90 / VERTICAL_RESOLUTION));

    const std::size_t VERTICAL_GRID_SIZE_1 = static_cast<std::size_t>(
        std::ceil(VERTICAL_ALIGNMENT_ANGLE_RADIUS / VERTICAL_RESOLUTION));
}