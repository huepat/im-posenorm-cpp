#include "HeaderContent.h"

#include <cstddef>
#include <format>
#include <map>
#include <vector>

#include "Property.h"

namespace IMPoseNorm::IO::In {

	HeaderContent::HeaderContent(
			std::size_t readerPosition,
			std::size_t vertexCount,
			std::size_t faceCount,
			PLYEncoding encoding,
			std::vector<Property> vertexProperties,
			std::vector<Property> faceProperties) :
				ReaderPosition(readerPosition),
				VertexCount(vertexCount),
				FaceCount(faceCount),
				Encoding(encoding),
				VertexProperties(vertexProperties),
				FaceProperties(faceProperties) {

		this->VertexPropertyLabelToIndexMapping = this->GetPropertyLabelToIndexMapping(vertexProperties);
		this->FacePropertyLabelToIndexMapping = this->GetPropertyLabelToIndexMapping(faceProperties);
	}

	void HeaderContent::CheckVertexProperty(
			const std::string& propertyLabel) const {

		this->CheckProperty(
			propertyLabel,
			"Vertex",
			this->VertexPropertyLabelToIndexMapping);
	}

	void HeaderContent::CheckFaceProperty(
			const std::string& propertyLabel) const {

		this->CheckProperty(
			propertyLabel,
			"Face",
			this->FacePropertyLabelToIndexMapping);
	}

	const Property& HeaderContent::GetVertexProperty(
			const std::string& propertyLabel) const {

		return this->VertexProperties[
			this->VertexPropertyLabelToIndexMapping.at(propertyLabel)
		];
	}

	const Property& HeaderContent::GetFaceProperty(
			const std::string& propertyLabel) const {

		return this->FaceProperties[
			this->FacePropertyLabelToIndexMapping.at(propertyLabel)
		];
	}

	std::map<std::string, std::size_t> HeaderContent::GetPropertyLabelToIndexMapping(
			const std::vector<Property>& properties) {

		std::map<std::string, std::size_t> propertyLabelToIndexMapping{};

		for (size_t i = 0; i < properties.size(); i++) {

			propertyLabelToIndexMapping[
				properties[i].Label
			] = i;
		}

		return propertyLabelToIndexMapping;
	}

	void HeaderContent::CheckProperty(
			const std::string& propertyLabel,
			const std::string& categoryLabel,
			const std::map<std::string, std::size_t>& propertyLabelToIndexMapping) const {

		if (!propertyLabelToIndexMapping.contains(propertyLabel)) {

			throw std::runtime_error(
				std::format(
					"{} property '{}' does not exist.",
					categoryLabel,
					propertyLabel));
		}
	}
}