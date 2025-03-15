#include <im-posenorm-lib/IO/PLYReaderWriterBase.h>

#include <im-posenorm-lib/IO/PLY.h>

#include <array>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace IMPoseNorm::IO {

	PLYReaderWriterBase::PLYReaderWriterBase() :
				faceVerticesPropertyLabel(DEFAULT_FACE_VERTICES_PROPERTY_LABEL ),
				coordinatePropertyLabels(DEFAULT_COORDINATE_PROPERTY_LABELS),
				normalPropertyLabels(DEFAULT_NORMAL_PROPERTY_LABELS),
				colorPropertyLabels(DEFAULT_COLOR_PROPERTY_LABELS) {
	}

	void PLYReaderWriterBase::SetFaceVerticesPropertyLabel(
			const std::string& faceVerticesPropertyLabel) {

		this->faceVerticesPropertyLabel = faceVerticesPropertyLabel;
	}

	void PLYReaderWriterBase::SetFaceVerticesPropertyLabel(
			std::string&& faceVerticesPropertyLabel) {

		this->faceVerticesPropertyLabel = std::move(faceVerticesPropertyLabel);
	}

	void PLYReaderWriterBase::SetCoordinatePropertyLabels(
			const std::array<std::string, 3>& coordinatePropertyLabels) {

		this->coordinatePropertyLabels = coordinatePropertyLabels;
	}

	void PLYReaderWriterBase::SetCoordinatePropertyLabels(
			std::array<std::string, 3>&& coordinatePropertyLabels) {

		this->coordinatePropertyLabels = std::move(coordinatePropertyLabels);
	}

	void PLYReaderWriterBase::SetNormalPropertyLabels(
			const std::array<std::string, 3>& normalPropertyLabels) {

		this->normalPropertyLabels = normalPropertyLabels;
	}

	void PLYReaderWriterBase::SetNormalPropertyLabels(
			std::array<std::string, 3>&& normalPropertyLabels) {

		this->normalPropertyLabels = std::move(normalPropertyLabels);
	}

	void PLYReaderWriterBase::SetColorPropertyLabels(
			const std::array<std::string, 3>& colorPropertyLabels) {

		this->colorPropertyLabels = colorPropertyLabels;
	}

	void PLYReaderWriterBase::SetColorPropertyLabels(
			std::array<std::string, 3>&& colorPropertyLabels) {

		this->colorPropertyLabels = std::move(colorPropertyLabels);
	}

	void PLYReaderWriterBase::CheckVertexPropertyUniqueness(
			bool hasNormals,
			bool hasColor,
			const std::vector<std::string>& additionalPropertyLabels) const {

		std::size_t referenceLabelCount = 3;

		if (hasNormals) {

			referenceLabelCount += 3;
		}

		if (hasColor) {

			referenceLabelCount += 3;
		}

		referenceLabelCount += additionalPropertyLabels.size();

		std::set<std::string> labels = this->GetVertexPropertyLabels(
			hasNormals,
			hasColor,
			additionalPropertyLabels);

		if (labels.size() != referenceLabelCount) {

			throw std::runtime_error("Property labels need to be unique.");
		}
	}

	std::set<std::string> PLYReaderWriterBase::GetVertexPropertyLabels(
			bool hasNormals,
			bool hasColor,
			const std::vector<std::string>& additionalPropertyLabels) const {

		std::set<std::string> labels{
			this->coordinatePropertyLabels[0],
			this->coordinatePropertyLabels[1],
			this->coordinatePropertyLabels[2]
		};

		if (hasNormals) {

			labels.insert(this->normalPropertyLabels[0]);
			labels.insert(this->normalPropertyLabels[1]);
			labels.insert(this->normalPropertyLabels[2]);
		}

		if (hasColor) {

			labels.insert(this->colorPropertyLabels[0]);
			labels.insert(this->colorPropertyLabels[1]);
			labels.insert(this->colorPropertyLabels[2]);
		}

		for (const std::string& additionalLabel : additionalPropertyLabels) {

			labels.insert(additionalLabel);
		}

		return labels;
	}
}