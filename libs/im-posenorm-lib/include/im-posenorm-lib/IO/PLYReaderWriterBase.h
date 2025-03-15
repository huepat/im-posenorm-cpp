#pragma once

#include <array>
#include <set>
#include <string>
#include <vector>

namespace IMPoseNorm::IO {

	class PLYReaderWriterBase {
	protected:
		std::string faceVerticesPropertyLabel;
		std::array<std::string, 3> coordinatePropertyLabels;
		std::array<std::string, 3> normalPropertyLabels;
		std::array<std::string, 3> colorPropertyLabels;

		PLYReaderWriterBase();

	public:
		void SetFaceVerticesPropertyLabel(
			const std::string& faceVerticesPropertyLabel);

		void SetFaceVerticesPropertyLabel(
			std::string&& faceVerticesPropertyLabel);

		void SetCoordinatePropertyLabels(
			const std::array<std::string, 3>& coordinatePropertyLabels);

		void SetCoordinatePropertyLabels(
			std::array<std::string, 3>&& coordinatePropertyLabels);

		void SetNormalPropertyLabels(
			const std::array<std::string, 3>& normalPropertyLabels);

		void SetNormalPropertyLabels(
			std::array<std::string, 3>&& normalPropertyLabels);

		void SetColorPropertyLabels(
			const std::array<std::string, 3>& colorPropertyLabels);

		void SetColorPropertyLabels(
			std::array<std::string, 3>&& colorPropertyLabels);

	protected:
		void CheckVertexPropertyUniqueness(
			bool hasNormals,
			bool hasColor,
			const std::vector<std::string>& additionalPropertyLabels) const;

		std::set<std::string> GetVertexPropertyLabels(
			bool hasNormals,
			bool hasColor,
			const std::vector<std::string>& additionalPropertyLabels) const;
	};
}