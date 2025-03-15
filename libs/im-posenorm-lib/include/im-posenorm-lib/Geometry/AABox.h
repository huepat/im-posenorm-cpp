#pragma once

#include <glm/glm.hpp>

namespace IMPoseNorm::Geometry {

	class Mesh;

	class AABox {

	private:
		glm::dvec3 min;
		glm::dvec3 max;

	public:
		AABox(
			const glm::dvec3& min,
			const glm::dvec3& max);

		AABox(
			glm::dvec3&& min,
			glm::dvec3&& max);

		glm::dvec3 GetSize() const;
		const glm::dvec3& GetMin() const;
		const glm::dvec3& GetMax() const;
		Mesh ConvertToMesh() const;
	};
}