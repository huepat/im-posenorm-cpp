#pragma once

#include <glm/glm.hpp>

#include <vector>

#include "HeaderContent.h"

namespace IMPoseNorm::IO::In {

	struct ShapeData {

	public:
		std::vector<glm::dvec3> Vertices;
		std::vector<glm::dvec3> Normals;
		std::vector<glm::u8vec3> Colors;
		std::vector<std::vector<float>> AdditionalProperties;
		std::vector<glm::u32vec3> Faces;

		ShapeData(
			bool useColor,
			std::size_t additionalPropertyCount,
			const HeaderContent& headerContent);
	};
}