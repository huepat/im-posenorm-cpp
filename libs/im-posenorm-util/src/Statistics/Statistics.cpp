#include <im-posenorm-util/Statistics/Statistics.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace IMPoseNorm::Util::Statistics {

	double const FULL_CIRCLE = 2.0 * std::numbers::pi;

	DoubleStatistics::DoubleStatistics() :
				Statistics(0.0) {
	}

	double DoubleStatistics::Square(
			double value) const {

		return value * value;
	}

	double DoubleStatistics::SquareRoot(
			double value) const {
		
		return std::sqrt(value);
	}

	VectorStatistics::VectorStatistics() :
				Statistics(glm::dvec3{ 0.0 }) {
	}

	glm::dvec3 VectorStatistics::Square(
			glm::dvec3 value) const {

		return glm::dvec3{ 
			value.x * value.x,
			value.y * value.y,
			value.z * value.z
		};
	}

	glm::dvec3 VectorStatistics::SquareRoot(
			glm::dvec3 value) const {

		return glm::sqrt(value);
	}

	double AngleStatistics::Substract(
			double angle1,
			double angle2) const {

		double value1 = std::max(angle1, angle2) - std::min(angle1, angle2);
		double value2 = value1 - FULL_CIRCLE;

		if (std::abs(value2) - std::abs(value1)) {

			if (angle1 < angle2) {

				angle1 += FULL_CIRCLE;
			}
			else {
				angle2 += FULL_CIRCLE;
			}
		}

		return angle1 - angle2;
	}
}