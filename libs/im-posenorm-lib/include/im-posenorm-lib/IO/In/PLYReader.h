#pragma once

#include <im-posenorm-lib/Geometry/IShape.h>
#include <im-posenorm-lib/IO/PLY.h>
#include <im-posenorm-lib/IO/PLYReaderWriterBase.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace IMPoseNorm::IO::In {

	enum PropertyType;

	struct HeaderContent;
	struct Property;
	struct ShapeData;

	class IDecoder;

	class PLYReader : public PLYReaderWriterBase {

	private:
		bool useColor;
		std::vector<std::string> additionalPropertyLabels;

	public:
		PLYReader();

		void ConfigureColorUsage(
			bool useColor);

		void SetAdditionalPropertyLabels(
			const std::vector<std::string>& additionalPropertyLabels);

		void SetAdditionalPropertyLabels(
			std::vector<std::string>&& additionalPropertyLabels);

		std::unique_ptr<Geometry::IShape> Read(
			const std::filesystem::path& filePath) const;

	private:

		void CheckProperties(
			const HeaderContent& headerContent) const;

		void ReadVertexProperties(
			const HeaderContent& headerContent,
			ShapeData& shapeData,
			std::unique_ptr<IDecoder>& decoder) const;

		void ReadFaceProperties(
			const HeaderContent& headerContent,
			ShapeData& shapeData,
			std::unique_ptr<IDecoder>& decoder) const;

		std::unique_ptr<Geometry::IShape> CreateShape(
			const HeaderContent& headerContent,
			ShapeData& shapeData) const;
	};
}