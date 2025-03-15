#include <im-posenorm-lib/IO/Out/PLYWriter.h>

#include <im-posenorm-lib/Geometry/IShape.h>
#include <im-posenorm-lib/IO/PLYReaderWriterBase.h>
#if IMPOSENORM_REPORT_TIMING
	#include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <array>
#include <filesystem>
#if IMPOSENORM_REPORT_TIMING
	#include <format>
#endif
#include <memory>
#include <utility>
#include <vector>

#include "IEncoder.h"

namespace IMPoseNorm::IO::Out {

	PLYWriter::PLYWriter(
			PLYEncoding encoding) :
				PLYReaderWriterBase(),
				writeCoordinatesAsFloat(false),
				encoding(encoding) {
	}

	void PLYWriter::SetWriteCoordinatesAsFloat() {

		this->writeCoordinatesAsFloat = true;
	}

	void PLYWriter::Write(
			const std::filesystem::path& filePath,
			const Geometry::IShape& shape) const {

#if IMPOSENORM_REPORT_TIMING
		Util::Time::TimeReporter timeReporter{};
#endif

		this->CheckVertexPropertyUniqueness(
			shape.HasNormals(),
			shape.HasColor(),
			shape.GetAdditionalPropertyLabels());

		std::unique_ptr<IEncoder> encoder = CreateEncoder(
			this->encoding,
			filePath);

		this->WriteHeader(
			filePath,
			shape);

		for (size_t pointIndex = 0; pointIndex < shape.GetPointCount(); pointIndex++) {

			encoder->Encode(
				shape.GetPoint(pointIndex),
				this->writeCoordinatesAsFloat);

			if (shape.HasPointNormals()) {

				encoder->EncodePropertySeperator();

				encoder->Encode(
					shape.GetNormal(pointIndex),
					this->writeCoordinatesAsFloat);
			}

			if (shape.HasColor()) {

				encoder->EncodePropertySeperator();

				encoder->Encode(
					shape.GetColor(pointIndex));
			}

			if (shape.HasAdditionalProperties()) {

				for (size_t propertyIndex = 0;
					propertyIndex < shape.GetAdditionalPropertyCount();
					propertyIndex++) {

					encoder->EncodePropertySeperator();

					encoder->Encode(
						shape.GetAdditionalProperty(
							pointIndex,
							propertyIndex));
				}
			}
			
			encoder->EncodeLineBreak();
		}

		for (size_t faceIndex = 0; faceIndex < shape.GetFaceCount(); faceIndex++) {

			encoder->Encode(
				shape.GetFace(faceIndex));

			encoder->EncodeLineBreak();
		}

		encoder->Flush();

#if IMPOSENORM_REPORT_TIMING
		timeReporter.Report(
			std::format(
				"Writing '{}'",
				filePath.string()));
#endif
	}

	void PLYWriter::WriteHeader(
			const std::filesystem::path& filePath,
			const Geometry::IShape& shape) const {
		
		std::string coordinateFormatLabel = this->writeCoordinatesAsFloat ?
			"float" :
			"double";

		std::ofstream file{
			filePath,
			std::ios::out
		};

		if (!file.is_open()) {

			throw std::runtime_error(
				std::format(
					"File '{}' cannot be opened.",
					filePath.string()));
		}

		file << "ply\n";

		file << std::format(
			"format {} 1.0\n",
			ToString(this->encoding));

		file << std::format(
			"element vertex {}\n",
			shape.GetPointCount());

		this->WritePropertyTriplet(
			file,
			coordinateFormatLabel,
			this->coordinatePropertyLabels);

		if (shape.HasPointNormals()) {

			this->WritePropertyTriplet(
				file,
				coordinateFormatLabel,
				this->normalPropertyLabels);
		}

		if (shape.HasColor()) {

			this->WritePropertyTriplet(
				file,
				"uchar",
				this->colorPropertyLabels);
		}

		if (shape.HasAdditionalProperties()) {

			std::vector<std::string> additionalPropertyLabels = shape.GetAdditionalPropertyLabels();

			for (size_t i = 0; i < additionalPropertyLabels.size(); i++) {

				file << std::format(
					"property float {}\n",
					additionalPropertyLabels[i]);
			}
		}

		file << std::format(
			"element face {}\n",
			shape.GetFaceCount());

		file << "property list uchar int vertex_indices\n";
		file << "end_header\n";

		file.close();
	}

	void PLYWriter::WritePropertyTriplet(
			std::ofstream& file,
			const std::string& propertyTypeLabel,
			const std::array<std::string, 3>& propertyLabels) const {

		file << std::format(
			"property {} {}\n",
			propertyTypeLabel,
			propertyLabels[0]);

		file << std::format(
			"property {} {}\n",
			propertyTypeLabel,
			propertyLabels[1]);

		file << std::format(
			"property {} {}\n",
			propertyTypeLabel,
			propertyLabels[2]);
	}
}