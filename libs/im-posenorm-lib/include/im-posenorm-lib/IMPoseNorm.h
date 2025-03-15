#pragma once

#include <im-posenorm-lib/Geometry/IShape.h>

#include <glm/glm.hpp>

namespace IMPoseNorm {

	const double DegreeToRadian(
		double degree);

	const double RadianToDegree(
		double radian);

	glm::dmat3 GetRotationAroundAxis(
		double rotation,
		const glm::dvec3& axis);

	glm::dmat3 NormalizePose(
		Geometry::IShape& shape,
		const glm::dvec3& upAxis,
		const glm::dvec3& horizontalAxis,
		bool useHorizontalPoseUnambiguation = false);
}