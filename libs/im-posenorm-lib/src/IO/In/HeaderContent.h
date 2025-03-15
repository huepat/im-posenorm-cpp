#pragma once

#include <im-posenorm-lib/IO/PLY.h>

#include <cstddef>
#include <map>
#include <vector>

#include "Property.h"

namespace IMPoseNorm::IO::In {

	struct HeaderContent {

	public:
		std::size_t ReaderPosition;
		std::size_t VertexCount;
		std::size_t FaceCount;
		PLYEncoding Encoding;
		std::vector<Property> VertexProperties;
		std::vector<Property> FaceProperties;
		std::map<std::string, std::size_t> VertexPropertyLabelToIndexMapping;
		std::map<std::string, std::size_t> FacePropertyLabelToIndexMapping;

		HeaderContent(
			std::size_t readerPosition,
			std::size_t vertexCount,
			std::size_t faceCount,
			PLYEncoding encoding,
			std::vector<Property> vertexProperties,
			std::vector<Property> faceProperties);

		void CheckVertexProperty(
			const std::string& propertyLabel) const;

		void CheckFaceProperty(
			const std::string& propertyLabel) const;

		const Property& GetVertexProperty(
			const std::string& propertyLabel) const;

		const Property& GetFaceProperty(
			const std::string& propertyLabel) const;

	private:
		std::map<std::string, std::size_t> GetPropertyLabelToIndexMapping(
			const std::vector<Property>& properties);

		void CheckProperty(
			const std::string& propertyLabel,
			const std::string& categoryLabel,
			const std::map<std::string, std::size_t>& propertyLabelToIndexMapping) const;
	};
}