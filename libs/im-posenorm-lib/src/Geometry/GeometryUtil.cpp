#include "GeometryUtil.h"

#include <im-posenorm-lib/Geometry/AABox.h>
#if IMPOSENORM_PARALLELIZE
	#include <im-posenorm-util/Parallel/Parallel.h>
#endif
#if IMPOSENORM_REPORT_TIMING
	#include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <glm/glm.hpp>

#if IMPOSENORM_PARALLELIZE
	#include <cstddef>
#endif
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace IMPoseNorm::Geometry {

	void Update(
			const glm::dvec3& point,
			glm::dvec3& min,
			glm::dvec3& max) {

		if (point.x < min.x) {
			min.x = point.x;
		}
		if (point.y < min.y) {
			min.y = point.y;
		}
		if (point.z < min.z) {
			min.z = point.z;
		}
		if (point.x > max.x) {
			max.x = point.x;
		}
		if (point.y > max.y) {
			max.y = point.y;
		}
		if (point.z > max.z) {
			max.z = point.z;
		}
	}

#if IMPOSENORM_PARALLELIZE
	struct MinMax {
	public:
		glm::dvec3 Min;
		glm::dvec3 Max;

		MinMax(
				glm::dvec3 min,
				glm::dvec3 max) : 
					Min(min),
					Max(max) {
		}
	};

	AABox GetBBox_(
			const std::vector<glm::dvec3>& points) {

		glm::dvec3 min{
			std::numeric_limits<double>::max()
		};
		glm::dvec3 max{
			std::numeric_limits<double>::min()
		};

		Util::Parallel::Parallel::For<MinMax>(
			static_cast<std::size_t>(0),
			points.size(),
			[&min, &max]() {
				return MinMax{
					min,
					max
				};
			},
			[&points](
				std::size_t partitionStartIndex,
				std::size_t partitionStopIndex,
				MinMax& partitionBounds) {

					for (size_t i = partitionStartIndex; i < partitionStopIndex; i++) {

						Update(
							points[i],
							partitionBounds.Min,
							partitionBounds.Max);
					}
			},
			[&min, &max](const MinMax& partitionBounds) {

				Update(
					partitionBounds.Min,
					min,
					max);

				Update(
					partitionBounds.Max,
					min,
					max);
			});

		return AABox{
			min,
			max
		};
	}
#else
	AABox GetBBox_(
			const std::vector<glm::dvec3>& points) {

		glm::dvec3 min{
			std::numeric_limits<double>::max()
		};
		glm::dvec3 max{
			std::numeric_limits<double>::min()
		};

		for (const glm::dvec3& point : points) {

			Update(
				point,
				min,
				max);
		}

		return AABox{
			min,
			max
		};
	}
#endif

	AABox GetBBox(
			const std::vector<glm::dvec3>& points) {
		
#if IMPOSENORM_REPORT_TIMING
		Util::Time::TimeReporter timeReporter{};
#endif

		AABox box = GetBBox_(points);

#if IMPOSENORM_REPORT_TIMING
		timeReporter.Report("BBox from Shape");
#endif

		return box;
	}
}