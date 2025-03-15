#pragma once

#include <im-posenorm-lib/IO/PLY.h>
#include <im-posenorm-lib/IO/PLYReaderWriterBase.h>

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <format>
#include <memory>
#include <stdexcept>

namespace IMPoseNorm::Geometry {
	class IShape;
}

namespace IMPoseNorm::IO::Out {

	class IEncoder;

	class PLYWriter : PLYReaderWriterBase {

	private:
		bool writeCoordinatesAsFloat;
		PLYEncoding encoding;

	public:
		PLYWriter(
			PLYEncoding encoding = PLYEncoding::BINARY_LITTLE_ENDIAN);

		void SetWriteCoordinatesAsFloat();

		void Write(
			const std::filesystem::path& filePath,
			const Geometry::IShape& shape) const;

	private:
		void WriteHeader(
			const std::filesystem::path& filePath,
			const Geometry::IShape& shape) const;

		void WritePropertyTriplet(
			std::ofstream& file,
			const std::string& propertyTypeLabel,
			const std::array<std::string, 3>& propertyLabels) const;
	};
}