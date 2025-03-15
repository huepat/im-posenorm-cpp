#include "ShapeData.h"

#include <glm/glm.hpp>

#include <vector>

#include "HeaderContent.h"

namespace IMPoseNorm::IO::In {

	ShapeData::ShapeData(
			bool useColor,
			std::size_t additionalPropertyCount,
			const HeaderContent& headerContent) :
				Vertices(headerContent.VertexCount),
				Normals(
					headerContent.FaceCount > 0 ?
						0 :
						headerContent.VertexCount),
				Colors(
					useColor ?
						headerContent.VertexCount :
						0),
				AdditionalProperties(additionalPropertyCount),
				Faces(headerContent.FaceCount) {
	}
}